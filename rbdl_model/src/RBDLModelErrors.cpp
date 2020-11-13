#include "rbdl_model/RBDLModelErrors.h"

namespace RBDLModel
{
namespace ModelErrors
{

RBDLModelError::RBDLModelError(std::string text): text(text) {}

const char* RBDLModelError::what() const noexcept
{
  return text.c_str();
}

RBDLModelInvalidFilePathError::RBDLModelInvalidFilePathError(std::string text) :RBDLModelError(text) {}
RBDLModelMissingParameterError::RBDLModelMissingParameterError(std::string text) :RBDLModelError(text) {}

//RBDLFileParseError::RBDLFileParseError(std::string text): RBDLError(text) {}

//RBDLDofMismatchError::RBDLDofMismatchError(std::string text): RBDLError(text) {}

//RBDLSizeMismatchError::RBDLSizeMismatchError(std::string text): RBDLError(
//    text) {}

//RBDLMissingImplementationError::RBDLMissingImplementationError(
//  std::string text): RBDLError(text) {}

//RBDLInvalidFileError::RBDLInvalidFileError(std::string text): RBDLError(text) {}

//RBDLInvalidParameterError::RBDLInvalidParameterError(std::string text):
//  RBDLError(text) {}

}
}
