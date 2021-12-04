//==============================================================================
//
//              Bounds - the Bounds class in the GPX library
//
//               Copyright (C) 2013  Dick van Oudheusden
//  
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free
// Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
//==============================================================================
// 
//  $Date: 2013-03-10 12:02:27 +0100 (Sun, 10 Mar 2013) $ $Revision: 5 $
//
//==============================================================================

#include "gpx/Metadata.h"


using namespace std;

namespace gpx
{
  Metadata::Metadata(Node *parent, const char *name, Node::Type type, bool mandatory) :
    Node(parent, name, type, mandatory),
    _name(this, "name", Node::ELEMENT, false),
    _desc(this, "desc", Node::ELEMENT, false),
    _author(this, "author", Node::ELEMENT, false),
    _copyright(this, "copyright", Node::ELEMENT, false),
    _time(this, "time", Node::ELEMENT, false),
    _keywords(this, "keywords", Node::ELEMENT, false),
    _bounds(this, "bounds", Node::ELEMENT, false),
    _extensions(this, "extensions", Node::ELEMENT, false),
    _links(this, "link", Node::ELEMENT, false)
  {
    getInterfaces().push_back(&_name);
    getInterfaces().push_back(&_desc);
    getInterfaces().push_back(&_author);
    getInterfaces().push_back(&_copyright);
    getInterfaces().push_back(&_time);
    getInterfaces().push_back(&_keywords);
    getInterfaces().push_back(&_bounds);
    getInterfaces().push_back(&_extensions);
    getInterfaces().push_back(&_links);
  }

  Metadata::~Metadata()
  {
  }
}
