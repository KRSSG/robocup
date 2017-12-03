#include "tactic_factory.h"

namespace Strategy {
  TacticFactory *TacticFactory::m_ptr = NULL;
  TacticFactory * TacticFactory::instance() {
    if (m_ptr == NULL)
      m_ptr = new TacticFactory();
    return m_ptr;
  }
  bool TacticFactory::RegCreateFn(const std::string & className, CreateFn fn) {
    registry[className] = fn;
    return true;
  }
  std::auto_ptr<Tactic> TacticFactory::Create(const std::string &className, int botID) const {
    FnRegistry::const_iterator regEntry = registry.find(className);
    if (regEntry != registry.end())
      return regEntry->second(botID);
    assert(0 && "Invalid tactic name (did you forget to register tactic?)"); // tried to create a Tactic which has not been registerd!
    return std::auto_ptr<Tactic>(NULL);
  }
}