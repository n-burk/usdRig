# FlatBuffers C++ runtime headers

The C++ runtime headers of [Google FlatBuffers](https://github.com/google/flatbuffers)
release **25.12.19**, copied **unmodified** from the tag `v25.12.19`
(commit `7e163021e59cca4f8e1e35a7c828b5c6b7915953`). They are the headers
that `include/flatbuffers/flatbuffers.h` includes, directly or through each
other, and nothing more:

- `include/flatbuffers/allocator.h`
- `include/flatbuffers/array.h`
- `include/flatbuffers/base.h`
- `include/flatbuffers/buffer.h`
- `include/flatbuffers/buffer_ref.h`
- `include/flatbuffers/default_allocator.h`
- `include/flatbuffers/detached_buffer.h`
- `include/flatbuffers/flatbuffer_builder.h`
- `include/flatbuffers/flatbuffers.h`
- `include/flatbuffers/stl_emulation.h`
- `include/flatbuffers/string.h`
- `include/flatbuffers/struct.h`
- `include/flatbuffers/table.h`
- `include/flatbuffers/vector.h`
- `include/flatbuffers/vector_downward.h`
- `include/flatbuffers/verifier.h`

`LICENSE` is the project's Apache License 2.0, unmodified. FlatBuffers is not
relicensed under the RigExec MIT license; see
[THIRD_PARTY_NOTICES.md](../../THIRD_PARTY_NOTICES.md).

## What uses them

The `.rigexec` schemas, `libs/rigExecBinary/rigexec.fbs` and
`libs/rigExecBinary/presentation.fbs`, are compiled by `flatc` 25.12.19 into
the checked-in headers under `libs/rigExecBinary/generated`. Those headers
`static_assert` this exact version, so the compiler and these headers are
updated together. The CMake target `rigExecFlatBuffers` puts this `include`
directory on the include path.

## Regenerating

`flatc` is not vendored. With a `flatc` 25.12.19 binary in `FLATC` or on
`PATH` (the release's own build, such as `flatc.exe` from its
`Windows.flatc.binary.zip`, or one built from the tag), run:

```
bin/gen_flatbuffers.sh        # POSIX shells
bin\gen_flatbuffers.bat       # Windows
```

Both run:

```
flatc --cpp --cpp-std c++17 --scoped-enums --gen-object-api \
      --object-prefix RigExecWire --object-suffix "" \
      --cpp-field-case-style lower --warnings-as-errors --reflect-types \
      --include-prefix rigExecBinary/generated/ --keep-prefix \
      -o libs/rigExecBinary/generated \
      libs/rigExecBinary/rigexec.fbs libs/rigExecBinary/presentation.fbs
```

`--reflect-types` adds the generated type tables (`FileTypeTable()` and the
others), which `RigExecFormatOpen` walks to bound a buffer before the
verifier reads it.

To move to another FlatBuffers version, replace these headers with the same
closure from the new release tag, unmodified, record the tag and its commit
here, update the version in the scripts and in the notices, and regenerate.
