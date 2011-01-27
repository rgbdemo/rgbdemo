#!/bin/sh
##
## configure_linux.sh
## Login : <nicolas.burrus@ensta.fr>
## Started on  Thu Feb 19 13:04:22 2009 Nicolas Burrus
## $Id$
## 
## Copyright (C) 2009, 2010 Nicolas Burrus
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program; if not, write to the Free Software
## Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
##

mkdir -p build || exit 1
cd build || exit 1

cmake .. \
    -Wno-dev \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_VERBOSE_MAKEFILE=1 \
    $*

echo "Program configured in directory build."
echo "Now go into build/ and run make."
