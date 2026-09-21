# Research papers and provenance

SARX is explicitly a research-integration project. This directory records the papers whose ideas are being connected, adapted, tested, or reserved for later integration.

## Redistribution policy

A public research repository should not silently republish copyrighted papers.

PDFs are committed under `papers/vendor/` only when the source provides a clear redistribution license compatible with redistribution of the unchanged paper. Where redistribution rights are unclear or the PDF itself says that server reposting/redistribution requires permission, SARX keeps the canonical citation and official/author-hosted PDF link here instead.

For convenient private/local study, run:

```bash
python papers/fetch_local.py
```

That downloads the linked research copies into `papers/local/`, which is intentionally gitignored.

## Core research lineage

| Work | Role in SARX | PDF / source | Repo status |
| --- | --- | --- | --- |
| McGraw, *Gram-Schmidt voxel constraints for real-time destructible soft bodies* (MIG 2024), DOI 10.1145/3677388.3696322 | Fast breakable voxel/PBD constraints; GPU-friendly fracture representation; direct ancestor of later McGraw work | Author PDF: https://web.ics.purdue.edu/~tmcgraw/papers/mcgraw_mig_2024.pdf | Linked only. The PDF itself contains an ACM redistribution notice, so SARX does not re-host it without unambiguous permission. |
| McGraw & Zhou, *Real-time voxelized mesh fracture with Gram-Schmidt constraints* (Computers & Graphics 2025), DOI 10.1016/j.cag.2025.104382 | Matured voxel fracture system, LOD constraints, fast partitioning and rendering pipeline | Author PDF: https://web.ics.purdue.edu/~tmcgraw/papers/voxel_2025.pdf | Linked only. Publisher access is closed and no clear redistribution license for the author-hosted PDF was verified. |
| Lin, *Real-Time Soft-Body Destruction with Multiresolution Structure* (Purdue M.S. thesis, 2025), DOI 10.25394/PGS.28724222 | Multiresolution/adaptive-resolution direction for destructible Gram-Schmidt soft bodies | Official record: https://hammer.purdue.edu/articles/thesis/Real-Time_Soft-Body_Destruction_with_Multiresolution_Structure/28724222 | **Vendored** under CC BY 4.0 as `papers/vendor/lin_2025_multires_soft_body.pdf`. |
| McGraw & Myers, *Real-Time Dissectible Deformable Models Using High-Performance Breakable Shape Constraints* (HPG 2026), DOI 10.1145/3820013 | Real-time cutting/tearing/excision; separation of coarse physical constraints from high-detail volumetric appearance | ACM/DOI: https://doi.org/10.1145/3820013 ; EG record: https://diglib.eg.org/items/02faf99b-d0d1-4ab5-992a-0919e2b0da50 | Linked only. Open access, but the redistribution terms are not being assumed to match the SARX repository license. |
| Benchekroun et al., *Fast Complementary Dynamics via Skinning Eigenmodes* (SIGGRAPH/TOG 2023), DOI 10.1145/3592404 | Rig-aware real-time secondary dynamics and reduced-space thinking; motivates separating animation authority from deformation detail | Project PDF: https://www.dgp.toronto.edu/projects/fast_complementary_dynamics_site/Fast_Complementary_Dynamics_Low_res.pdf | Linked only. The PDF explicitly says server reposting/redistribution requires permission. |
| Chang et al., *Lifting the Winding Number: Precise Discontinuities in Neural Fields for Physics Simulation* (SIGGRAPH 2025), DOI 10.1145/3721238.3730597 | Topology/discontinuity-aware reduced representation; relevant to later thin tissue, membranes, skin, cloth, and render/simulation surrogates | Author PDF: https://www.dgp.toronto.edu/projects/windlifter/assets/windlifter.pdf | Linked only pending a redistribution-license verification. |
| Macklin, Müller & Chentanez, *XPBD: Position-Based Simulation of Compliant Constrained Dynamics* (MIG 2016), DOI 10.1145/2994258.2994272 | Mathematical basis for SARX compliant structural, attachment, and tetrahedral constraints | DOI: https://doi.org/10.1145/2994258.2994272 ; author copy commonly hosted at http://mmacklin.com/xpbd.pdf | Linked only; free access is not the same as a blanket redistribution license. |
| Müller et al., *Position Based Dynamics* (JVCIR 2007), DOI 10.1016/j.jvcir.2007.01.005 | Foundational PBD formulation | DOI: https://doi.org/10.1016/j.jvcir.2007.01.005 | Linked only. |

## What SARX does and does not claim

SARX does **not** claim that the cited papers are components of one pre-existing method. It is an independent research prototype exploring the missing interfaces between ideas that were demonstrated separately:

- rig-driven animation and physics-driven deformation,
- high-performance breakable soft-body topology,
- explicit volumetric constraints,
- real-time cutting and persistent discontinuities,
- adaptive/multiresolution activation,
- render/simulation representation decoupling.

Where SARX uses a standard technique directly (for example XPBD), the source is credited. Where SARX introduces glue that is not claimed by a source paper (for example breakable animation authority, dynamic rig islands, deterministic cross-system fracture events, and the current CPU reference semantics), that distinction is stated in the main README.

See `CITATIONS.bib` for machine-readable citations.
