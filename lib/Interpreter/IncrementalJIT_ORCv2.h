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

#include "llvm/ADT/FunctionExtras.h"
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

using ReadyForUnloadingCallback =
    llvm::unique_function<void(const llvm::Module* M)>;

class IncrementalJIT {
public:
  IncrementalJIT(IncrementalExecutor& Executor,
                 std::unique_ptr<llvm::TargetMachine> TM,
                 std::unique_ptr<llvm::orc::ExecutorProcessControl> EPC,
                 ReadyForUnloadingCallback NotifyReadyForUnloading,
                 llvm::Error &Err);

  // FIXME: Accept a LLVMContext as well, e.g. the one that was used for the
  // particular module in Interpreter, CIFactory or BackendPasses (would be
  // more efficient)
  void addModule(std::unique_ptr<llvm::Module> M);

  llvm::Expected<std::unique_ptr<llvm::Module>>
  removeModule(const llvm::Module* M);

  /// Get the address of a symbol from the JIT or (optionally) the host process.
  /// Use this to resolve symbols based on their IR names (as they are coming
  /// from clang's mangler).
  uint64_t getSymbolAddress(const std::string& Name, bool AlsoInProcess);

  /// Returns the address of the symbol and a boolean value that doesn't appear
  /// to be use anywhere in the public cling codebase.
  ///
  /// FIXME: This is used for both, to inject external symbols and to lookup
  /// exiting symbols. Should we use the above `getSymbolAddress()` for the
  /// former and change this into something like `addDefinition()`?
  ///
  std::pair<void*, bool> lookupSymbol(llvm::StringRef LinkerMangledName,
                                      void* KnownAddr = nullptr,
                                      bool ReplaceExisting = false);

private:
  std::unique_ptr<llvm::orc::LLJIT> Jit;
  llvm::orc::SymbolMap InjectedSymbols;

  /// FIXME: If the relation between modules and transactions is a bijection, the
  /// mapping via module pointers here is unnecessary. The transaction should
  /// store the resource tracker directly and pass it to `remove()` for
  /// unloading.
  std::map<const llvm::Module *, llvm::orc::ResourceTrackerSP> ResourceTrackers;
  std::map<const llvm::Module *, llvm::orc::ThreadSafeModule> CompiledModules;

  // FIXME: Remove this once we simplified code unloading with ORCv2
  ReadyForUnloadingCallback NotifyReadyForUnloading;

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
