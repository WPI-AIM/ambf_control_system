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

}
}
