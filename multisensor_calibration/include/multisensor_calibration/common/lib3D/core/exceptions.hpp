// Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef LIB3D_EXCEPTIONS_HPP
#define LIB3D_EXCEPTIONS_HPP

// std
#include <exception>
#include <string>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define FN_NAME __FUNCTION__
#elif defined(__linux__) || defined(__unix__)
#define FN_NAME __PRETTY_FUNCTION__
#endif

namespace lib3d
{

//==================================================================================================
/**
 * @brief Exception for providing an invalid arguments to a function
 */
class InvalidArgumentException : public std::exception
{

  public:
    /**
     @brief Constructor
     @param[in] iCallerFn Name of caller function.
     @param[in] iMsg Additional message to print.
     */
    explicit InvalidArgumentException(std::string iCallerFn = "",
                                      std::string iMsg      = "") :
      std::exception(),
      mMessage(new std::string("lib3d::InvalidArgumentException"))
    {
        if (!iMsg.empty())
        {
            mMessage->append(": " + iMsg);
        }

        if (!iCallerFn.empty())
        {
            mMessage->append(" in function " + iCallerFn);
        }

        mMessage->append(".\n");
    }

    ~InvalidArgumentException()
    {
        if (mMessage)
            delete mMessage;
    }

    virtual const char* what() const noexcept
    {
        return mMessage->c_str();
    }

    std::string* mMessage;
};

} // namespace lib3d

#endif // LIB3D_EXCEPTIONS_HPP
