///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015-2016 Double Negative Visual Effects
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of Double Negative Visual Effects nor the names
// of its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
// LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
//
///////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <bitset>
#include <climits>
#include <cassert>

#include <boost/scoped_array.hpp>

#include <openvdb/openvdb.h>
#include <openvdb_points/openvdb.h>
#include <openvdb_points/tools/PointDataGrid.h>
#include <openvdb_points/tools/PointCount.h>

using namespace openvdb;
using namespace tools;

template <typename T>
T rotateLeftComplement(T value)
{
    const unsigned int mask(CHAR_BIT * sizeof(value) - 1);
    const T allones(T(1) & mask);
    return (~value << allones) | (value >> ((-allones) & mask));
}

template <typename T>
T rotateRightComplement(T value)
{
    const unsigned int mask(CHAR_BIT * sizeof(value) - 1);
    const T allones(T(1) & mask);
    return (~value >> allones) | (value << ((-allones) & mask));
}

int
main(int argc, char *argv[])
{

    uint8_t value = -10;

    std::bitset<8> x(value);
    std::cerr << x << std::endl;

    uint8_t value_invert = ~value;

    std::bitset<8> x_invert(value_invert);
    std::cerr << x_invert << std::endl;

    uint8_t value2 = rotateLeftComplement<uint8_t>(value);

    std::bitset<8> x2(value2);
    std::cerr << x2 << std::endl;

    uint8_t value3 = rotateRightComplement<uint8_t>(value2);

    std::bitset<8> x3(value3);
    std::cerr << x3 << std::endl;

    std::vector<int> values;

    const int size = 10;

    for (int i = 0; i < size; i++)
    {
        values.push_back(i);
    }

    uint bits = ((size * 3) + 7) / 8;

    std::cerr << "Bits: " << bits << std::endl;

    boost::scoped_array<char> key(new char[bits]);

    initialize();
    points::initialize();

    io::File file("/tmp/caches/test.vdb");
    file.open();

    GridBase::Ptr baseGrid = file.readGrid("points");
    PointDataGrid::ConstPtr grid = GridBase::grid<PointDataGrid>(baseGrid);

    file.close();

    io::File newFile("/tmp/caches/newtest.vdb");

    GridCPtrVec grids;
    grids.push_back(grid);

    newFile.write(grids);

    newFile.close();

    return 0;
}

// Copyright (c) 2015-2016 Double Negative Visual Effects
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
