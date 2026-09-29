// Copyright (c) Meta Platforms, Inc. and affiliates.
// SPDX-License-Identifier: MIT
// Source: facebookexperimental/noodles, ff5d473f10e8c37ceaf0da11ea7cb80805bc8314.
// License: plugin/usdNoodles/NOODLES_LICENSE.txt.

#version 330 core

in vec2 vTexCoord;
out vec4 FragColor;

uniform sampler2D uTexture;

void main() {
    FragColor = texture(uTexture, vTexCoord);
}
