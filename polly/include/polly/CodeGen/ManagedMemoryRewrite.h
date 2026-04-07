//===---- ManagedMemoryRewrite.cpp - Rewrite global & malloc'd memory -----===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Take a module and rewrite:
// 1. `malloc` -> `polly_mallocManaged`
// 2. `free` -> `polly_freeManaged`
// 3. global arrays with initializers -> global arrays that are initialized
//                                       with a constructor call to
//                                       `polly_mallocManaged`.
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_MANAGEDMEMORYREWRITE_H
#define POLLY_MANAGEDMEMORYREWRITE_H

#include "polly/CodeGen/PPCGCodeGeneration.h"

using namespace llvm;
using namespace polly;

bool runManagedMemoryRewrite(Function &F, GPUArch Arch, GPURuntime Runtime);

#endif // POLLY_MANAGEDMEMORYREWRITE_H
