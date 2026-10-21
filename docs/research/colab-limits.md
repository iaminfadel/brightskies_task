# Colab limits for a self-hosted CARLA runner

**Issue:** #3 · **Date:** 2026-10-21 · **Scope:** consumer Colab (free and Google AI Plus), GitHub Actions, CARLA 0.9.15.

## Executive answer

Colab is a disposable notebook VM, not a persistent host. Google deletes VMs when idle, enforces an unpublished maximum lifetime, and publishes a 12-hour general session ceiling; only Colab Pro+ documents 24 hours. Google AI Plus is not Colab Pro: it grants 50 compute units (CCUs) and priority accelerators, but no background-execution entitlement is documented for Plus. Only Google AI Ultra is documented as getting "uninterrupted background execution." A GitHub Actions self-hosted runner is the wrong shape for this platform: it would spend most of its life idle waiting for jobs, has no documented public inbound endpoint, loses its workspace when the VM is reclaimed, and would need re-registering on every boot. On a no-GPU VM, CARLA 0.9.15's live smoke test is not supported: CARLA requires a 6–8 GB GPU and Vulkan, and in no-rendering mode GPU sensors return empty data. A self-hosted runner on a repository that may become public is also a hard security no.

**Recommendation:** do not register Colab as a persistent runner. At most, use Colab as an ephemeral batch target via the official `colab run` CLI; otherwise use a real GPU host.

## Evidence

| Question | Finding | Source | Key quote |
|---|---|---|---|
| Lifecycle and idle policy | VMs are deleted after idleness; maximum lifetime is unpublished | Colab FAQ [1] | "Virtual machines are deleted when idle for a while"; "Colab does not publish these limits" |
| Session length | 12 hours general; 24 hours on Pro+ | Colab FAQ [1] | "notebooks can run for at most 12 hours"; Pro+ supports "continuous code execution for up to 24 hours" |
| Background execution | Pro+ up to 24h; Google AI Ultra "uninterrupted"; no Plus entitlement documented | FAQ [1], Ultra help [4], launch post [5] | "Google AI Ultra subscribers also unlock uninterrupted background execution" |
| SSH / remote control | Free tier without positive CCU: disallowed. Paid plans remove this restriction while units remain positive. All tiers ban remote proxies | Colab FAQ [1] | "remote control such as SSH shells, remote desktops"; "connecting to remote proxies" |
| CI / runner use | No explicit CI authorization; all tiers ban non-interactive web-service offerings; interactive compute is prioritized | FAQ [1], Additional Terms [2] | "other web service offerings not related to interactive compute with Colab"; "Colab prioritizes interactive compute" |
| Google AI Plus | 50 CCUs, web-only, paid plan, not a trial; separate from Colab Pro and units stack | Plus help [3], FAQ [1] | "Google AI Plus members get 50 CCUs"; a Google AI plan and a Colab-native subscription "can be active ... at the same time" |
| Storage | VM disk is ephemeral; Drive is persistent but does not enlarge the VM disk; Plus has 400 GB or 2 TB Drive | FAQ [1], Plus help [3], plans page [6] | "purchasing more space on Drive will not increase the amount of disk available on Colab VMs" |
| Network | Outbound only; no public inbound endpoint documented; official SSH is an outbound WebSocket | CLI docs [11], FAQ [1] | "SSH-over-WebSocket"; no documented public ingress |
| GitHub runner risk | Public repos are a hard no; private runners still lack ephemerality guarantees | GitHub secure-use [7], runner docs [8] | "self-hosted runners should almost never be used for public repositories" |
| CARLA requirements | 20 GB disk, 6–8 GB GPU, Vulkan; no-render mode returns empty sensor data | CARLA quickstart [9], rendering docs [10] | "server needs at least a 6 GB GPU"; "CARLA will use about 20 GB"; "only supports the Vulkan graphics API" |
| Colab CLI | Official ephemeral job runner: fresh VM, run, collect outputs, tear down | Colab CLI [11] | "Provision a fresh VM, execute ... retrieve output files, and automatically tear down" |

## What this means for the proposed runner

- **It is not a VM you keep.** Even a paid runtime is reclaimed on idle or at the session ceiling. A runner that waits for jobs is idle by definition, so it will die before work arrives.
- **Plus does not buy background execution.** The Plus benefits page lists 50 CCUs, not background execution. Google assigns uninterrupted background execution to Ultra only. Treat closing the browser as ending a Plus session.
- **Free-tier remote control and worker automation are explicitly prohibited.** "Remote control" and "distributed computing workers" are barred without a positive CCU balance. Paid plans lift that specific restriction, but a persistent runner is still not an authorized CI product.
- **No inbound means no webhook-driven runner.** Colab has no documented public ingress. A runner only needs outbound HTTPS, but boot-on-demand requires an external dispatcher; Colab cannot receive the GitHub job webhook itself.
- **Local disk is disposable.** A 20 GB CARLA install must be repeated per VM or fetched from external storage. Artifacts must be uploaded before the VM is reclaimed; a kill mid-upload loses them.
- **No GPU, no live CARLA test.** The proposed no-GPU runtime can at best run a no-rendering startup/API test; cameras and GPU sensors return empty data. CARLA's supported path needs a 6–8 GB GPU and Vulkan.
- **Public repo is a security hard stop.** GitHub warns that untrusted workflow code can persistently compromise a self-hosted runner and exfiltrate secrets. If there is any chance the repo becomes public, do not attach one.

## Safe operating envelope

| Constraint | Safe setting |
|---|---|
| Role | Ephemeral batch job, never a registered runner |
| Trigger | External dispatcher or manual; no inbound webhook to Colab |
| Lifetime | One job per VM; target under 2 hours; hard ceiling 12 hours |
| Plan | Plus: 50 CCUs; monitor usage; assume no background execution |
| Network | Outbound only; no public SSH, reverse tunnel, or exposed port |
| Repository | Private only; no public repo; no long-lived secrets; JIT runner if unavoidable |
| Storage | Upload artifacts to GitHub or external object storage; assume `/content` is lost |
| CARLA | GPU 6–8 GB for live tests; CPU only for no-rendering startup/API checks |
| Security | Dedicated account, least privilege, no repo-write token by default |

## Fallback options

1. **Recommended: local or on-prem GPU host as a self-hosted runner.** Private repo, JIT registration, firewall, persistent CARLA install, no Colab limits.
2. **Cloud GPU ephemeral runner per job.** GCP/AWS with a prebuilt CARLA image; artifacts to object storage; no always-on runner.
3. **Hybrid CI.** GitHub-hosted runners for build/lint/unit tests; a GPU host only for CARLA nightly or manual smoke tests.
4. **Colab-only fallback.** Use official `colab run` on a GPU runtime, launched by a trusted dispatcher, one job per VM, outputs fetched before teardown. Treat it as occasional batch compute, not CI.
5. **Managed alternative.** Colab Enterprise or Vertex AI Workbench if GCP billing is acceptable. The FAQ's old "dedicated VM at GCP Marketplace" escape hatch is stale: that listing was deprecated on 2025-03-21 [12].

## Open questions

- Is the Plus Colab benefit active on this account/region? Trial memberships are excluded [3].
- What idle timeout and maximum lifetime apply? Google does not publish them [1].
- Does Plus receive any background execution beyond the documented 50 CCUs? Current docs say Ultra only [4][5].
- What is the CCU consumption rate for CPU-only and GPU runtimes? No conversion is published.
- What VM disk and RAM does a Plus-backed runtime get, and will a 20 GB CARLA install fit?
- Does `colab ssh` / the CLI work with Plus, or does it require Pro/Pro+? Not documented.
- Can the runtime reach GitHub and artifact endpoints reliably? No egress allowlist or static IP is published.
- Does CARLA 0.9.15 run on the current Colab Ubuntu image with software Vulkan? Must be tested.
- Is a CI runner "interactive compute" under the Terms? Seek written clarification.

## Sources

1. Google Colab FAQ — https://research.google.com/colaboratory/faq.html — accessed 2026-10-21.
2. Google Colab Additional Terms of Service — https://colab.research.google.com/terms — last modified 2026-08-13; accessed 2026-10-21.
3. Google One Help, "Use Google AI Plus benefits" — https://support.google.com/googleone/answer/16882689 — accessed 2026-10-21.
4. Google One Help, "Use Google AI Ultra benefits" — https://support.google.com/googleone/answer/16286513 — accessed 2026-10-21.
5. Google Developers Blog, "Colab is now part of your Google AI plan" — https://developers.googleblog.com/colab-is-now-part-of-your-google-ai-plan/ — published 2026-09-22.
6. Google AI plans — https://one.google.com/about/google-ai-plans/ — accessed 2026-10-21.
7. GitHub Docs, "Secure use reference" — https://docs.github.com/en/actions/reference/security/secure-use — accessed 2026-10-21.
8. GitHub Docs, "Adding self-hosted runners" — https://docs.github.com/en/actions/how-tos/manage-runners/self-hosted-runners/add-runners — accessed 2026-10-21.
9. CARLA 0.9.15 quickstart — https://carla.readthedocs.io/en/0.9.15/start_quickstart/ — accessed 2026-10-21.
10. CARLA 0.9.15 rendering options — https://carla.readthedocs.io/en/0.9.15/adv_rendering_options/ — accessed 2026-10-21.
11. googlecolab/google-colab-cli — https://github.com/googlecolab/google-colab-cli — accessed 2026-10-21; see `docs/06_ssh_access.md`.
12. Colab GCP Marketplace — https://research.google.com/colaboratory/marketplace.html — accessed 2026-10-21; "deprecated on March 21, 2025."
