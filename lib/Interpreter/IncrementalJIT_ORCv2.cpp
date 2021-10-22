//--------------------------------------------------------------------*- C++ -*-
// CLING - the C++ LLVM-based InterpreterG :)
// author:  Stefan Gränitz <stefan.graenitz@gmail.com>
//
// This file is dual-licensed: you can choose to license it under the University
// of Illinois Open Source License or the GNU Lesser General Public License. See
// LICENSE.TXT for details.
//------------------------------------------------------------------------------

#include "IncrementalJIT_ORCv2.h"

#include <llvm/ExecutionEngine/Orc/JITTargetMachineBuilder.h>
#include <llvm/ExecutionEngine/Orc/ObjectLinkingLayer.h>
#include "llvm/IR/LLVMContext.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;
using namespace llvm::orc;

namespace cling {

IncrementalJIT::IncrementalJIT(IncrementalExecutor& Executor,
                               std::unique_ptr<TargetMachine> TM,
                               std::unique_ptr<llvm::orc::ExecutorProcessControl> EPC,
                               NotifyCompiledCallback NCF,
                               Error &Err)
    : TM(std::move(TM)),
      SingleThreadedContext(std::make_unique<LLVMContext>()) {
  ErrorAsOutParameter _(&Err);

  // FIXME: We should probably take codegen settings from the CompilerInvocation
  // and not from the target machine
  JITTargetMachineBuilder JTMB(this->TM->getTargetTriple());
  JTMB.setCodeModel(this->TM->getCodeModel());
  JTMB.setCodeGenOptLevel(this->TM->getOptLevel());
  JTMB.setFeatures(this->TM->getTargetFeatureString());
  JTMB.setRelocationModel(this->TM->getRelocationModel());

  LLJITBuilder Builder;
  Builder.setJITTargetMachineBuilder(std::move(JTMB));
  Builder.setExecutorProcessControl(std::move(EPC));

  // FIXME: In LLVM 13 this only works for ELF and MachO platforms
  Builder.setObjectLinkingLayerCreator(
      [&](ExecutionSession &ES, const Triple &TT) {
        return std::make_unique<ObjectLinkingLayer>(
            ES, std::make_unique<jitlink::InProcessMemoryManager>());
      });

  if (Expected<std::unique_ptr<LLJIT>> JitInstance = Builder.create()) {
    Jit = std::move(*JitInstance);
  } else {
    Err = JitInstance.takeError();
    return;
  }

  // FIXME: Handle NotifyCompiledCallback

  // FIXME: Wire up host process symbol lookup
}

VModuleKey IncrementalJIT::addModule(std::unique_ptr<Module> M) {
  const Module *RawModulePtr = M.get();

  ThreadSafeModule TSM(std::move(M), SingleThreadedContext);
  if (Error Err = Jit->addIRModule(std::move(TSM))) {
    logAllUnhandledErrors(std::move(Err), errs(),
                          "IncrementalJIT::addModule failed: ");
    return 0;
  }

  // cling module names appear to follow the cling-module-X scheme, where X is
  // is a counter. So they should be unique. We could always hash the module
  // contents instead to obtain a unique ID.
  return GlobalValue::getGUID(RawModulePtr->getName());
}

std::pair<void*, bool> IncrementalJIT::lookupSymbol(StringRef LinkerMangledName,
                                                    void* KnownAddr,
                                                    bool ReplaceExisting) {
  Expected<JITEvaluatedSymbol> Symbol =
      Jit->lookupLinkerMangled(LinkerMangledName);
  if (!Symbol && !KnownAddr) {
    logAllUnhandledErrors(Symbol.takeError(), errs(),
                          "IncrementalJIT::lookupSymbol failed: ");
    return std::make_pair(nullptr, false);
  }

  if (KnownAddr) {
    if (!Symbol) {
      consumeError(Symbol.takeError());
    } else if (Symbol && !ReplaceExisting) {
      errs() << "IncrementalJIT::lookupSymbol failed: "
             << "cannot redefine existing symbol '" << LinkerMangledName
             << "'\n";
      return std::make_pair(nullptr, false);
    }

    bool Inserted;
    SymbolMap::iterator It;
    std::tie(It, Inserted) = InjectedSymbols.try_emplace(
        Jit->getExecutionSession().intern(LinkerMangledName),
        JITEvaluatedSymbol::fromPointer(KnownAddr));
    assert(Inserted && "Why wasn't this found in the initial Jit lookup?");

    if (Error Err = Jit->getMainJITDylib().define(absoluteSymbols({*It}))) {
      logAllUnhandledErrors(std::move(Err), errs(),
                            "IncrementalJIT::lookupSymbol failed: ");
      return std::make_pair(nullptr, false);
    }

    return std::make_pair(KnownAddr, true);
  }

  errs() << "IncrementalJIT::lookupSymbol failed: not yet implemented\n";
  return std::make_pair(nullptr, false);
}

uint64_t IncrementalJIT::getSymbolAddress(const std::string& Name,
                                          bool AlsoInProcess) {
  // TODO: Is the AlsoInProcess parameter still in use? It looks like it didn't
  // actually work as expected in the ORCv1 implementation.
  Expected<JITEvaluatedSymbol> Symbol = Jit->lookup(Name);
  if (!Symbol) {
    logAllUnhandledErrors(Symbol.takeError(), errs(),
                          "IncrementalJIT::lookupSymbol failed: ");
    return 0;
  }

  return Symbol->getAddress();
}

} // namespace cling
