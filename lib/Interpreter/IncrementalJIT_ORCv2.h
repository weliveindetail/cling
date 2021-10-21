//--------------------------------------------------------------------*- C++ -*-
// CLING - the C++ LLVM-based InterpreterG :)
// author:  Stefan Gränitz <stefan.graenitz@gmail.com>
//
// This file is dual-licensed: you can choose to license it under the University
// of Illinois Open Source License or the GNU Lesser General Public License. See
// LICENSE.TXT for details.
//------------------------------------------------------------------------------

#ifndef CLING_INCREMENTAL_JIT_ORCV2_H
#define CLING_INCREMENTAL_JIT_ORCV2_H

#include "IncrementalJIT.h"

#include "llvm/ADT/StringRef.h"
#include "llvm/IR/Module.h"
#include "llvm/ExecutionEngine/Orc/ExecutorProcessControl.h"
#include "llvm/Support/Error.h"
#include "llvm/Target/TargetMachine.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace cling {

class IncrementalExecutor;

class IncrementalJIT {
public:
  IncrementalJIT(IncrementalExecutor& Executor,
                 std::unique_ptr<llvm::TargetMachine> TM,
                 std::unique_ptr<llvm::orc::ExecutorProcessControl> EPC,
                 llvm::orc::NotifyCompiledCallback NCF) {}

  llvm::orc::VModuleKey addModule(std::unique_ptr<llvm::Module> M) {
    return 0;
  }

  llvm::Error removeModule(const llvm::Module* M) {
    return llvm::Error::success();
  }

  uint64_t getSymbolAddress(const std::string& Name, bool AlsoInProcess) {
    return 0;
  }

  std::pair<void*, bool>
  lookupSymbol(llvm::StringRef Name, void* Addr = nullptr, bool Jit = false) {
    return std::make_pair(nullptr, false);
  }
};

} // namespace cling

#endif // CLING_INCREMENTAL_JIT_ORCV2_H
