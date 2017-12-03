#pragma once

#include "tactics/tactic.h"
#include <map>
#include <string>

namespace Strategy {
  // a singleton class to create a tactic object, given the tactic's name.
  // reference: http://www.drdobbs.com/conversations-abstract-factory-template/184403786
  class TacticFactory {
    typedef std::auto_ptr<Tactic> (*CreateFn) (int botID);
    typedef std::map<std::string, CreateFn> FnRegistry;
    FnRegistry registry;
    TacticFactory() {}
    TacticFactory(const TacticFactory&); // Not Implemented to protect singleton status
    TacticFactory & operator=(const TacticFactory &); // Not Implemented.
    static TacticFactory *m_ptr;
  public:
    static TacticFactory* instance();
    bool RegCreateFn(const std::string&, CreateFn);
    std::auto_ptr<Tactic> Create(const std::string &, int botID) const;
  };

  // MACRO DEFINITION FOR REGISTERING A TACTIC
  #ifndef REGISTER_TACTIC
  #define REGISTER_TACTIC(DERIVED_CLASS) \
namespace \
{ \
    std::auto_ptr<Tactic> Create##DERIVED_CLASS(int botID) \
    { \
        return std::auto_ptr<Tactic>(new DERIVED_CLASS(botID)); \
    } \
    bool dummy=TacticFactory::instance()->RegCreateFn( \
        #DERIVED_CLASS, Create##DERIVED_CLASS); \
} 
#endif
}