/*-
 * Copyright (c) 2026 Alchemilla Ventures Private Limited
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer
 *    in this position and unchanged.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * virtio balloon device emulation.
 * Provides a mechanism for the host to request the guest to give up memory.
 */

#include <sys/param.h>
#ifndef WITHOUT_CAPSICUM
#include <sys/capsicum.h>
#endif
#include <sys/linker_set.h>
#include <sys/mman.h>

#include <assert.h>
#include <err.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "bhyverun.h"
#include "debug.h"
#include "pci_emul.h"
#include "virtio.h"
#ifdef BHYVE_SNAPSHOT
#include "snapshot.h"
#endif

#include <dev/virtio/balloon/virtio_balloon.h>

#define VTBALLOON_RINGSZ 64

static int pci_vtballoon_debug;
#define DPRINTF(params)          \
	if (pci_vtballoon_debug) \
	PRINTLN params
#define WPRINTF(params) PRINTLN params

struct pci_vtballoon_softc {
	struct virtio_softc vbs_vs;
	struct vqueue_info vbs_vq[2];
	pthread_mutex_t vbs_mtx;
	uint32_t vbs_num_pages;
	uint32_t vbs_actual;
};

#define VBS_INFLATE_VQ 0
#define VBS_DEFLATE_VQ 1

static void pci_vtballoon_reset(void *);
static void pci_vtballoon_notify(void *, struct vqueue_info *);
static int pci_vtballoon_cfgread(void *, int, int, uint32_t *);
static int pci_vtballoon_cfgwrite(void *, int, int, uint32_t);
#ifdef BHYVE_SNAPSHOT
static int pci_vtballoon_snapshot(void *, struct vm_snapshot_meta *);
#endif

static struct virtio_consts vtballoon_vi_consts = {
	.vc_name = "vtballoon",
	.vc_nvq = 2,
	.vc_cfgsize = sizeof(struct virtio_balloon_config),
	.vc_reset = pci_vtballoon_reset,
	.vc_qnotify = pci_vtballoon_notify,
	.vc_cfgread = pci_vtballoon_cfgread,
	.vc_cfgwrite = pci_vtballoon_cfgwrite,
	.vc_hv_caps = 0,
#ifdef BHYVE_SNAPSHOT
	.vc_snapshot = pci_vtballoon_snapshot,
#endif
};

static void
pci_vtballoon_reset(void *vsc)
{
	struct pci_vtballoon_softc *sc;

	sc = vsc;

	DPRINTF(("vtballoon: device reset requested"));
	vi_reset_dev(&sc->vbs_vs);
	sc->vbs_num_pages = 0;
	sc->vbs_actual = 0;
}

static void
pci_vtballoon_notify(void *vsc, struct vqueue_info *vq)
{
	struct iovec iov;
	struct pci_vtballoon_softc *sc;
	struct vi_req req;
	uint32_t *pfns;
	void *hva;
	uint64_t gpa;
	int n, i, npfns;

	sc = vsc;

	if (vq->vq_num == VBS_INFLATE_VQ) {
		while (vq_has_descs(vq)) {
			n = vq_getchain(vq, &iov, 1, &req);
			assert(n == 1);

			npfns = iov.iov_len / sizeof(uint32_t);
			pfns = (uint32_t *)iov.iov_base;

			DPRINTF(
			    ("vtballoon: inflate request for %d pages", npfns));

			for (i = 0; i < npfns; i++) {
				gpa = (uint64_t)pfns[i] << VIRTIO_BALLOON_PFN_SHIFT;
				DPRINTF(
				    ("vtballoon:   PFN %d: 0x%x (gpa 0x%lx)", i, pfns[i], gpa));
				hva = paddr_guest2host(sc->vbs_vs.vs_pi->pi_vmctx, gpa, PAGE_SIZE);
				if (hva != NULL)
					madvise(hva, PAGE_SIZE, MADV_DONTNEED);
			}

			sc->vbs_actual += npfns;
			vq_relchain(vq, req.idx, 0);
		}
		vq_endchains(vq, 1);
	} else if (vq->vq_num == VBS_DEFLATE_VQ) {
		while (vq_has_descs(vq)) {
			n = vq_getchain(vq, &iov, 1, &req);
			assert(n == 1);

			npfns = iov.iov_len / sizeof(uint32_t);
			pfns = (uint32_t *)iov.iov_base;

			DPRINTF(
			    ("vtballoon: deflate request for %d pages", npfns));

			for (i = 0; i < npfns; i++) {
				DPRINTF(
				    ("vtballoon:   PFN %d: 0x%x", i, pfns[i]));
			}

			if (sc->vbs_actual >= (uint32_t)npfns)
				sc->vbs_actual -= npfns;
			else
				sc->vbs_actual = 0;

			vq_relchain(vq, req.idx, 0);
		}
		vq_endchains(vq, 1);
	} else {
		DPRINTF(("vtballoon: notify on unknown queue %d", vq->vq_num));
	}
}

static int
pci_vtballoon_cfgread(void *vsc, int offset, int size, uint32_t *val)
{
	struct pci_vtballoon_softc *sc;

	sc = vsc;

	if (offset == offsetof(struct virtio_balloon_config, num_pages)) {
		*val = sc->vbs_num_pages;
	} else if (offset == offsetof(struct virtio_balloon_config, actual)) {
		*val = sc->vbs_actual;
	} else {
		*val = 0;
	}

	DPRINTF(("vtballoon: config read offset %d size %d -> 0x%x", offset,
	    size, *val));

	return (0);
}

static int
pci_vtballoon_cfgwrite(void *vsc, int offset, int size, uint32_t val)
{
	struct pci_vtballoon_softc *sc;

	sc = vsc;

	if (offset == offsetof(struct virtio_balloon_config, num_pages)) {
		sc->vbs_num_pages = val;
		DPRINTF(("vtballoon: desired pages set to %d", val));
	} else if (offset == offsetof(struct virtio_balloon_config, actual)) {
		DPRINTF(("vtballoon: attempt to write actual (ignored)"));
	} else {
		DPRINTF(("vtballoon: unknown config write offset %d size %d",
		    offset, size));
	}

	return (0);
}

#ifdef BHYVE_SNAPSHOT
static int
pci_vtballoon_snapshot(void *vsc, struct vm_snapshot_meta *meta)
{
	int ret;
	struct pci_vtballoon_softc *sc = vsc;

	DPRINTF(("vtballoon: device snapshot requested"));

	SNAPSHOT_VAR_OR_LEAVE(sc->vbs_num_pages, meta, ret, done);
	SNAPSHOT_VAR_OR_LEAVE(sc->vbs_actual, meta, ret, done);

done:
	return (ret);
}
#endif

static int
pci_vtballoon_init(struct pci_devinst *pi, nvlist_t *nvl __unused)
{
	struct pci_vtballoon_softc *sc;

	sc = calloc(1, sizeof(struct pci_vtballoon_softc));

	pthread_mutex_init(&sc->vbs_mtx, NULL);

	vi_softc_linkup(&sc->vbs_vs, &vtballoon_vi_consts, sc, pi, sc->vbs_vq);
	sc->vbs_vs.vs_mtx = &sc->vbs_mtx;

	sc->vbs_vq[VBS_INFLATE_VQ].vq_qsize = VTBALLOON_RINGSZ;
	sc->vbs_vq[VBS_DEFLATE_VQ].vq_qsize = VTBALLOON_RINGSZ;

	pci_set_cfgdata16(pi, PCIR_DEVICE, VIRTIO_DEV_BALLOON);
	pci_set_cfgdata16(pi, PCIR_VENDOR, VIRTIO_VENDOR);
	pci_set_cfgdata8(pi, PCIR_CLASS, PCIC_MEMORY);
	pci_set_cfgdata16(pi, PCIR_SUBDEV_0, VIRTIO_ID_BALLOON);
	pci_set_cfgdata16(pi, PCIR_SUBVEND_0, VIRTIO_VENDOR);

	if (vi_intr_init(&sc->vbs_vs, 1, fbsdrun_virtio_msix()))
		return (1);
	vi_set_io_bar(&sc->vbs_vs, 0);

	return (0);
}

static const struct pci_devemu pci_de_vtballoon = {
	.pe_emu = "virtio-balloon",
	.pe_init = pci_vtballoon_init,
	.pe_barwrite = vi_pci_write,
	.pe_barread = vi_pci_read,
#ifdef BHYVE_SNAPSHOT
	.pe_snapshot = vi_pci_snapshot,
#endif
};
PCI_EMUL_SET(pci_de_vtballoon);
