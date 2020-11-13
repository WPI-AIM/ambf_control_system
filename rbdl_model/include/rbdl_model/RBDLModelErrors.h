#ifndef RBDLMODELERRORS_H
#define RBDLMODELERRORS_H

#include <string>
#include <exception>


namespace RBDLModel
{

/** Namespace that contains error classes that may be thrown by RBDL Model creation with blender YAML file as input*/
namespace ModelErrors
{

/** \brief Base class for all RBDL Model creattion exceptions
 *
 * The base class for all errors thrown in RBDL Model creation.
 * It saves an error message that then can be read when catching the
 * error in the calling code, by calling the what() function.
 *
 * When catching for this class you will also be catching any other error
 * types derived from this class.
 *
 * To add another error type just subclass this class, you only need to implement
 * the constructor and call the parent constructor with your error message.
 *
 */
class RBDLModelError : public std::exception
{
protected:
  std::string text;
public:
  RBDLModelError(std::string text);
  virtual const char* what() const noexcept;
};


/** \brief Thrown if file not found
 *
 * This error class is thrown if a fle supplied not found
 *
 */
class RBDLModelInvalidFilePathError : public RBDLModelError
{
public:
  RBDLModelInvalidFilePathError(std::string text);
};

/** \brief Thrown if file not found
 *
 * This error class is thrown if a fle supplied not found
 *
 */
class RBDLModelMissingParameterError : public RBDLModelError
{
public:
  RBDLModelMissingParameterError(std::string text);
};


}
}
#endif // RBDLMODELERRORS_H
