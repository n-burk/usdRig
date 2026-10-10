LZMA SDK 26.04, Igor Pavlov, public domain.
Official https://www.7-zip.org/sdk.html
Archive https://github.com/ip7z/7zip/releases/download/26.04/lzma2604.7z
SHA256 BD98058D0E12C5DA970DD4E5E1D8FA24F2D704914D96E69C28F92ED45421A439
Original license preserved in DOC/lzma-sdk.txt. Only codec sources and required headers are included.
Local patch: LzFind.c wraps SIMD selection in RIGEXEC_LZMA_PORTABLE_SCALAR guard; build selects existing scalar normalization, removing mutable CPU-dispatch pointer. All other SDK files unchanged.
