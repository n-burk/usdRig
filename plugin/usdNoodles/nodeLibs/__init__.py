#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

#

"""
Node Libraries Package

Provides concrete implementations of node libraries for the usdNoodles system.
"""


from .usdPrimLibrary import UsdPrimLibrary

__all__ = [
    "UsdPrimLibrary",
    ]
