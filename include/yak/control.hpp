/* vim:set ts=4 sw=4 sts=4 et: */
/*
 * The MIT License (MIT)
 * Copyright (c) 2014 Tamas Nepusz
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef YAK_CONTROL_HPP
#define YAK_CONTROL_HPP

#include <Eigen/Core>

namespace yak {

/**
 * Simple class that implements an n-dimensional control vector.
 */
template <int dimension, typename T=double>
class SimpleControlVector {
public:
    /**
     * Static constant that can be used to retrieve the dimensions of the
     * control vector at compile time.
     */
    static const int DIMENSIONS = dimension;

    /**
     * Typedef for the datatype used in this class.
     */
    typedef T DataType;

    /**
     * Typedef for a column vector containing exactly \c DIMENSIONS coordinates.
     */
    typedef Eigen::Matrix<T, dimension, 1> ColumnVector;
    
    /**
     * The value of the control vector.
     */
    ColumnVector value;

    /**
     * Default constructor.
     */
    SimpleControlVector() : value(ColumnVector::Zero()) {}
};

}       // end of namespaces

#endif
