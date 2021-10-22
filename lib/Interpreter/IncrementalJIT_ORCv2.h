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
#include "llvm/ExecutionEngine/Orc/LLJIT.h"
#include "llvm/ExecutionEngine/Orc/ThreadSafeModule.h"
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
                 llvm::orc::NotifyCompiledCallback NCF,
                 llvm::Error &Err);

  // FIXME: Accept a LLVMContext as well, e.g. the one that was used for the
  // particular module in Interpreter, CIFactory or BackendPasses (would be
  // more efficient)
  llvm::orc::VModuleKey addModule(std::unique_ptr<llvm::Module> M);

  llvm::Error removeModule(const llvm::Module* M) {
    return llvm::Error::success();
  }

  /// Get the address of a symbol from the JIT or (optionally) the host process.
  /// Use this to resolve symbols based on their IR names (as they are coming
  /// from clang's mangler).
  uint64_t getSymbolAddress(const std::string& Name, bool AlsoInProcess);

  std::pair<void*, bool>
  lookupSymbol(llvm::StringRef Name, void* Addr = nullptr, bool Jit = false) {
    return std::make_pair(nullptr, false);
  }

private:
  std::unique_ptr<llvm::orc::LLJIT> Jit;

  // FIXME: Move TargetMachine ownership to BackendPasses
  std::unique_ptr<llvm::TargetMachine> TM;

  // TODO: We only need the context for materialization. Instead of defining it
  // here we might want to pass one in on a per-module basis.
  //
  // FIXME: Using a single context for all modules prevents concurrent
  // compilation.
  //
  llvm::orc::ThreadSafeContext SingleThreadedContext;
};

} // namespace cling

#endif // CLING_INCREMENTAL_JIT_ORCV2_H
