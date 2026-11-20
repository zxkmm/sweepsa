# Licensing

Copyright (c) 2026 zxkmm.

The source code in this repository is dual-licensed.

## 1. GNU AGPL-3.0 (default)

You may use, modify and redistribute it under the GNU Affero General Public
License, version 3 only. The full text is in [LICENSE](LICENSE). In short, if
you distribute this software, or a work based on it, or let people use a
modified version over a network, you must make the complete corresponding
source of your version available under the AGPL-3.0 too.

## 2. Commercial licence

If you want to use sweepsa in a product or service without the AGPL-3.0's
obligations (for example, without publishing your own source code), you can
buy a commercial licence.

Contact: gh@zxkmm.com

## Contributions

Because of the commercial option, contributions can only be accepted if the
contributor agrees that the maintainer may distribute them under both
licences above. Opening a pull request means you agree to this.
<!-- TODO: decide whether a formal CLA is needed instead of this clause. -->

## Third-party components

The submodules under `third_party/` and `gaintables/adi_1300_4000_wide.txt`
keep their own licences:

| component | licence |
|---|---|
| Dear ImGui | MIT |
| ImPlot | MIT |
| GLFW | zlib |
| PFFFT | BSD-like (FFTPACK) |
| ADI no-OS gain table | BSD-3-Clause (`gaintables/LICENSE-adi-no-OS.txt`) |

The commercial licence covers only code whose copyright zxkmm holds. These
components remain under their own (permissive) licences either way.

## Binary releases

The prebuilt archives on the Releases page include `sweepd`, which is closed
source and not covered by this repository's licences. See `LICENSE.txt` in
each archive and in this repository.
