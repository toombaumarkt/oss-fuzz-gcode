/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

\******************************************************************************/

#pragma once


#include "Token.h"

#include <cbang/parse/Tokenizer.h>


namespace GCode {
  class Tokenizer : public cb::Tokenizer<TokenType> {
  public:
    using cb::Tokenizer<TokenType>::Token_T;

    Tokenizer(const cb::SmartPointer<cb::Scanner> &scanner) :
      cb::Tokenizer<TokenType>(scanner) {advance();}

    Tokenizer(const cb::InputSource &source) :
      cb::Tokenizer<TokenType>(new cb::Scanner(source)) {advance();}

    bool isID(const std::string &id) const;

  protected:
    void comment();
    void parenComment();
    void number(bool positive = true);
    void id();

    // From cb::Tokenizer
    void next();
  };
}
