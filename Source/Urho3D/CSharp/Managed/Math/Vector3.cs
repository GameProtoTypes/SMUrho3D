/*
Copyright (c) 2006 - 2008 The Open Toolkit library.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

using System;
using System.Runtime.InteropServices;
using System.Xml.Serialization;

namespace Urho3DNet
{
    /// <summary>
    /// Represents a 3D vector using three single-precision floating-point numbers.
    /// </summary>
    /// <remarks>
    /// The Vector3 structure is suitable for interoperation with unmanaged code requiring three consecutive floats.
    /// </remarks>
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct Vector3 : IEquatable<Vector3>
    {
        /// <summary>
        /// The X component of the Vector3.
        /// </summary>
        public float X;

        /// <summary>
        /// The Y component of the Vector3.
        /// </summary>
        public float Y;

        /// <summary>
        /// The Z component of the Vector3.
        /// </summary>
        public float Z;

        /// <summary>
        /// Constructs a new instance.
        /// </summary>
        /// <param name="value">The value that will initialize this instance.</param>
        public Vector3(float value)
        {
            X = value;
            Y = value;
            Z = value;
        }

        /// <summary>
        /// Constructs a new Vector3.
        /// </summary>
        /// <param name="x">The x component of the Vector3.</param>
        /// <param name="y">The y component of the Vector3.</param>
        /// <param name="z">The z component of the Vector3.</param>
        public Vector3(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        /// <summary>
        /// Constructs a new Vector3 from the given Vector2.
        /// </summary>
        /// <param name="v">The Vector2 to copy components from.</param>
        public Vector3(in Vector2 v, float z = 0.0f)
        {
            X = v.X;
            Y = v.Y;
            Z = z;
        }

        /// <summary>
        /// Constructs a new Vector3 from the given Vector3.
        /// </summary>
        /// <param name="v">The Vector3 to copy components from.</param>
        public Vector3(in Vector3 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        /// <summary>
        /// Constructs a new Vector3 from the given Vector4.
        /// </summary>
        /// <param name="v">The Vector4 to copy components from.</param>
        public Vector3(in Vector4 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        /// <summary>
        /// Convert vector to System.Numerics.Vector3 type.
        /// </summary>
        /// <param name="vector">A vector to convert.</param>
        /// <returns></returns>
        public static implicit operator System.Numerics.Vector3(in Vector3 vector)
        {
            return new System.Numerics.Vector3(vector.X, vector.Y, vector.Z);
        }

        /// <summary>
        /// Gets or sets the value at the index of the Vector.
        /// </summary>
        public float this[int index] {
            get{
                if (index == 0)
                {
                    return X;
                }
                else if (index == 1)
                {
                    return Y;
                }
                else if (index == 2)
                {
                    return Z;
                }
                throw new IndexOutOfRangeException("You tried to access this vector at index: " + index);
            } set{
                if (index == 0)
                {
                    X = value;
                }
                else if (index == 1)
                {
                    Y = value;
                }
                else if (index == 2)
                {
                    Z = value;
                }
                else
                {
                    throw new IndexOutOfRangeException("You tried to set this vector at index: " + index);
                }
            }
        }

        /// <summary>
        /// Gets the length (magnitude) of the vector.
        /// </summary>
        /// <see cref="LengthFast"/>
        /// <seealso cref="LengthSquared"/>
        public float Length
        {
            get
            {
                return (float)System.Math.Sqrt(X * X + Y * Y + Z * Z);
            }
        }

        /// <summary>
        /// Gets an approximation of the vector length (magnitude).
        /// </summary>
        /// <remarks>
        /// This property uses an approximation of the square root function to calculate vector magnitude, with
        /// an upper error bound of 0.001.
        /// </remarks>
        /// <see cref="Length"/>
        /// <seealso cref="LengthSquared"/>
        public float LengthFast
        {
            get
            {
                return 1.0f / MathDefs.InverseSqrtFast(X * X + Y * Y + Z * Z);
            }
        }

        /// <summary>
        /// Gets the square of the vector length (magnitude).
        /// </summary>
        /// <remarks>
        /// This property avoids the costly square root operation required by the Length property. This makes it more suitable
        /// for comparisons.
        /// </remarks>
        /// <see cref="Length"/>
        /// <seealso cref="LengthFast"/>
        public float LengthSquared
        {
            get
            {
                return X * X + Y * Y + Z * Z;
            }
        }

        /// <summary>
        /// Returns a copy of the Vector3 scaled to unit length.
        /// </summary>
        public Vector3 Normalized()
        {
            Vector3 v = this;
            v.Normalize();
            return v;
        }

        /// <summary>
        /// Scales the Vector3 to unit length.
        /// </summary>
        public void Normalize()
        {
            float scale = 1.0f / this.Length;
            X *= scale;
            Y *= scale;
            Z *= scale;
        }

        /// <summary>
        /// Scales the Vector3 to approximately unit length.
        /// </summary>
        public void NormalizeFast()
        {
            float scale = MathDefs.InverseSqrtFast(X * X + Y * Y + Z * Z);
            X *= scale;
            Y *= scale;
            Z *= scale;
        }

        /// <summary>
        /// Defines a unit-length Vector3 that points towards the X-axis.
        /// </summary>
        public static readonly Vector3 UnitX = new Vector3(1, 0, 0);

        /// <summary>
        /// Defines a unit-length Vector3 that points towards the Y-axis.
        /// </summary>
        public static readonly Vector3 UnitY = new Vector3(0, 1, 0);

        /// <summary>
        /// Defines a unit-length Vector3 that points towards the Z-axis.
        /// </summary>
        public static readonly Vector3 UnitZ = new Vector3(0, 0, 1);

        /// <summary>
        /// Defines a zero-length Vector3.
        /// </summary>
        public static readonly Vector3 Zero = new Vector3(0, 0, 0);

        /// <summary>
        /// Defines an instance with all components set to 1.
        /// </summary>
        public static readonly Vector3 One = new Vector3(1, 1, 1);

        public static readonly Vector3 Up = new Vector3(0,  1, 0);
        public static readonly Vector3 Down = new Vector3(0, -1, 0);
        public static readonly Vector3 Forward = new Vector3(0, 0,  1);
        public static readonly Vector3 Back = new Vector3(0, 0, -1);
        public static readonly Vector3 Left = new Vector3(-1, 0, 0);
        public static readonly Vector3 Right = new Vector3( 1, 0, 0);

        /// <summary>
        /// Defines the size of the Vector3 struct in bytes.
        /// </summary>
        public static readonly int SizeInBytes = Marshal.SizeOf(new Vector3());

        /// <summary>
        /// Adds two vectors.
        /// </summary>
        /// <param name="a">Left operand.</param>
        /// <param name="b">Right operand.</param>
        /// <returns>Result of operation.</returns>
        public static Vector3 Add(Vector3 a, in Vector3 b)
        {
            Add(a, b, out a);
            return a;
        }

        /// <summary>
        /// Adds two vectors.
        /// </summary>
        /// <param name="a">Left operand.</param>
        /// <param name="b">Right operand.</param>
        /// <param name="result">Result of operation.</param>
        public static void Add(in Vector3 a, in Vector3 b, out Vector3 result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
        }

        /// <summary>
        /// Subtract one Vector from another
        /// </summary>
        /// <param name="a">First operand</param>
        /// <param name="b">Second operand</param>
        /// <returns>Result of subtraction</returns>
        public static Vector3 Subtract(Vector3 a, in Vector3 b)
        {
            Subtract(a, b, out a);
            return a;
        }

        /// <summary>
        /// Subtract one Vector from another
        /// </summary>
        /// <param name="a">First operand</param>
        /// <param name="b">Second operand</param>
        /// <param name="result">Result of subtraction</param>
        public static void Subtract(in Vector3 a, in Vector3 b, out Vector3 result)
        {
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
            result.Z = a.Z - b.Z;
        }

        /// <summary>
        /// Multiplies a vector by a scalar.
        /// </summary>
        /// <param name="vector">Left operand.</param>
        /// <param name="scale">Right operand.</param>
        /// <returns>Result of the operation.</returns>
        public static Vector3 Multiply(Vector3 vector, float scale)
        {
            Multiply(vector, scale, out vector);
            return vector;
        }

        /// <summary>
        /// Multiplies a vector by a scalar.
        /// </summary>
        /// <param name="vector">Left operand.</param>
        /// <param name="scale">Right operand.</param>
        /// <param name="result">Result of the operation.</param>
        public static void Multiply(in Vector3 vector, float scale, out Vector3 result)
        {
            result.X = vector.X * scale;
            result.Y = vector.Y * scale;
            result.Z = vector.Z * scale;
        }

        /// <summary>
        /// Multiplies a vector by the components a vector (scale).
        /// </summary>
        /// <param name="vector">Left operand.</param>
        /// <param name="scale">Right operand.</param>
        /// <returns>Result of the operation.</returns>
        public static Vector3 Multiply(Vector3 vector, in Vector3 scale)
        {
            Multiply(vector, scale, out vector);
            return vector;
        }

        /// <summary>
        /// Multiplies a vector by the components of a vector (scale).
        /// </summary>
        /// <param name="vector">Left operand.</param>
        /// <param name="scale">Right operand.</param>
        /// <param name="result">Result of the operation.</param>
        public static void Multiply(in Vector3 vector, in Vector3 scale, out Vector3 result)
        {
            result.X = vector.X * scale.X;
            result.Y = vector.Y * scale.Y;
            result.Z = vector.Z * scale.Z;
        }

        /// <summary>
        /// Divides a vector by a scalar.
        /// </summary>
        /// <param name="vector">Left operand.</param>
        /// <param name="scale">Right operand.</param>
        /// <returns>Result of the operation.</returns>
        public static Vector3 Divide(Vector3 vector, float scale)
        {
            Divide(vector, scale, out vector);
            return vector;
        }

        /// <summary>
        /// Divides a vector by a scalar.
        /// </summary>
        /// <param name="vector">Left operand.</param>
        /// <param name="scale">Right operand.</param>
        /// <param name="result">Result of the operation.</param>
        public static void Divide(in Vector3 vector, float scale, out Vector3 result)
        {
            result.X = vector.X / scale;
            result.Y = vector.Y / scale;
            result.Z = vector.Z / scale;
        }

        /// <summary>
        /// Divides a vector by the components of a vector (scale).
        /// </summary>
        /// <param name="vector">Left operand.</param>
        /// <param name="scale">Right operand.</param>
        /// <returns>Result of the operation.</returns>
        public static Vector3 Divide(Vector3 vector, in Vector3 scale)
        {
            Divide(vector, scale, out vector);
            return vector;
        }

        /// <summary>
        /// Divide a vector by the components of a vector (scale).
        /// </summary>
        /// <param name="vector">Left operand.</param>
        /// <param name="scale">Right operand.</param>
        /// <param name="result">Result of the operation.</param>
        public static void Divide(in Vector3 vector, in Vector3 scale, out Vector3 result)
        {
            result.X = vector.X / scale.X;
            result.Y = vector.Y / scale.Y;
            result.Z = vector.Z / scale.Z;
        }

        /// <summary>
        /// Returns a vector created from the smallest of the corresponding components of the given vectors.
        /// </summary>
        /// <param name="a">First operand</param>
        /// <param name="b">Second operand</param>
        /// <returns>The component-wise minimum</returns>
        public static Vector3 ComponentMin(Vector3 a, in Vector3 b)
        {
            a.X = a.X < b.X ? a.X : b.X;
            a.Y = a.Y < b.Y ? a.Y : b.Y;
            a.Z = a.Z < b.Z ? a.Z : b.Z;
            return a;
        }

        /// <summary>
        /// Returns a vector created from the smallest of the corresponding components of the given vectors.
        /// </summary>
        /// <param name="a">First operand</param>
        /// <param name="b">Second operand</param>
        /// <param name="result">The component-wise minimum</param>
        public static void ComponentMin(in Vector3 a, in Vector3 b, out Vector3 result)
        {
            result.X = a.X < b.X ? a.X : b.X;
            result.Y = a.Y < b.Y ? a.Y : b.Y;
            result.Z = a.Z < b.Z ? a.Z : b.Z;
        }

        /// <summary>
        /// Returns a vector created from the largest of the corresponding components of the given vectors.
        /// </summary>
        /// <param name="a">First operand</param>
        /// <param name="b">Second operand</param>
        /// <returns>The component-wise maximum</returns>
        public static Vector3 ComponentMax(Vector3 a, in Vector3 b)
        {
            a.X = a.X > b.X ? a.X : b.X;
            a.Y = a.Y > b.Y ? a.Y : b.Y;
            a.Z = a.Z > b.Z ? a.Z : b.Z;
            return a;
        }

        /// <summary>
        /// Returns a vector created from the largest of the corresponding components of the given vectors.
        /// </summary>
        /// <param name="a">First operand</param>
        /// <param name="b">Second operand</param>
        /// <param name="result">The component-wise maximum</param>
        public static void ComponentMax(in Vector3 a, in Vector3 b, out Vector3 result)
        {
            result.X = a.X > b.X ? a.X : b.X;
            result.Y = a.Y > b.Y ? a.Y : b.Y;
            result.Z = a.Z > b.Z ? a.Z : b.Z;
        }

        /// <summary>
        /// Returns the Vector3 with the minimum magnitude. If the magnitudes are equal, the second vector
        /// is selected.
        /// </summary>
        /// <param name="left">Left operand</param>
        /// <param name="right">Right operand</param>
        /// <returns>The minimum Vector3</returns>
        public static Vector3 MagnitudeMin(in Vector3 left, in Vector3 right)
        {
            return left.LengthSquared < right.LengthSquared ? left : right;
        }

        /// <summary>
        /// Returns the Vector3 with the minimum magnitude. If the magnitudes are equal, the second vector
        /// is selected.
        /// </summary>
        /// <param name="left">Left operand</param>
        /// <param name="right">Right operand</param>
        /// <param name="result">The magnitude-wise minimum</param>
        /// <returns>The minimum Vector3</returns>
        public static void MagnitudeMin(in Vector3 left, in Vector3 right, out Vector3 result)
        {
            result = left.LengthSquared < right.LengthSquared ? left : right;
        }

        /// <summary>
        /// Returns the Vector3 with the maximum magnitude. If the magnitudes are equal, the first vector
        /// is selected.
        /// </summary>
        /// <param name="left">Left operand</param>
        /// <param name="right">Right operand</param>
        /// <returns>The maximum Vector3</returns>
        public static Vector3 MagnitudeMax(in Vector3 left, in Vector3 right)
        {
            return left.LengthSquared >= right.LengthSquared ? left : right;
        }

        /// <summary>
        /// Returns the Vector3 with the maximum magnitude. If the magnitudes are equal, the first vector
        /// is selected.
        /// </summary>
        /// <param name="left">Left operand</param>
        /// <param name="right">Right operand</param>
        /// <param name="result">The magnitude-wise maximum</param>
        /// <returns>The maximum Vector3</returns>
        public static void MagnitudeMax(in Vector3 left, in Vector3 right, out Vector3 result)
        {
            result = left.LengthSquared >= right.LengthSquared ? left : right;
        }

        /// <summary>
        /// Per-component min of two 3-vectors.
        /// </summary>
        /// <param name="left">Left operand</param>
        /// <param name="right">Right operand</param>
        /// <returns>The minimum Vector3</returns>
        public static Vector3 Min(in Vector3 left, in Vector3 right)
        {
            return new Vector3(Math.Min(left.X, right.X), Math.Min(left.Y, right.Y), Math.Min(left.Z, right.Z));
        }

        /// <summary>
        /// Per-component max of two 3-vectors.
        /// </summary>
        /// <param name="left">Left operand</param>
        /// <param name="right">Right operand</param>
        /// <returns>The minimum Vector3</returns>
        public static Vector3 Max(in Vector3 left, in Vector3 right)
        {
            return new Vector3(Math.Max(left.X, right.X), Math.Max(left.Y, right.Y), Math.Max(left.Z, right.Z));
        }

        /// <summary>
        /// Clamp a vector to the given minimum and maximum vectors
        /// </summary>
        /// <param name="vec">Input vector</param>
        /// <param name="min">Minimum vector</param>
        /// <param name="max">Maximum vector</param>
        /// <returns>The clamped vector</returns>
        public static Vector3 Clamp(Vector3 vec, in Vector3 min, in Vector3 max)
        {
            vec.X = vec.X < min.X ? min.X : vec.X > max.X ? max.X : vec.X;
            vec.Y = vec.Y < min.Y ? min.Y : vec.Y > max.Y ? max.Y : vec.Y;
            vec.Z = vec.Z < min.Z ? min.Z : vec.Z > max.Z ? max.Z : vec.Z;
            return vec;
        }

        /// <summary>
        /// Clamp a vector to the given minimum and maximum vectors
        /// </summary>
        /// <param name="vec">Input vector</param>
        /// <param name="min">Minimum vector</param>
        /// <param name="max">Maximum vector</param>
        /// <param name="result">The clamped vector</param>
        public static void Clamp(in Vector3 vec, in Vector3 min, in Vector3 max, out Vector3 result)
        {
            result.X = vec.X < min.X ? min.X : vec.X > max.X ? max.X : vec.X;
            result.Y = vec.Y < min.Y ? min.Y : vec.Y > max.Y ? max.Y : vec.Y;
            result.Z = vec.Z < min.Z ? min.Z : vec.Z > max.Z ? max.Z : vec.Z;
        }

        /// <summary>
        /// Compute the euclidean distance between two vectors.
        /// </summary>
        /// <param name="vec1">The first vector</param>
        /// <param name="vec2">The second vector</param>
        /// <returns>The distance</returns>
        public static float Distance(in Vector3 vec1, in Vector3 vec2)
        {
            float result;
            Distance(vec1, vec2, out result);
            return result;
        }

        /// <summary>
        /// Compute the euclidean distance between two vectors.
        /// </summary>
        /// <param name="vec1">The first vector</param>
        /// <param name="vec2">The second vector</param>
        /// <param name="result">The distance</param>
        public static void Distance(in Vector3 vec1, in Vector3 vec2, out float result)
        {
            result = (float)Math.Sqrt((vec2.X - vec1.X) * (vec2.X - vec1.X) + (vec2.Y - vec1.Y) * (vec2.Y - vec1.Y) + (vec2.Z - vec1.Z) * (vec2.Z - vec1.Z));
        }

        /// <summary>
        /// Compute the squared euclidean distance between two vectors.
        /// </summary>
        /// <param name="vec1">The first vector</param>
        /// <param name="vec2">The second vector</param>
        /// <returns>The squared distance</returns>
        public static float DistanceSquared(in Vector3 vec1, in Vector3 vec2)
        {
            float result;
            DistanceSquared(vec1, vec2, out result);
            return result;
        }

        /// <summary>
        /// Compute the squared euclidean distance between two vectors.
        /// </summary>
        /// <param name="vec1">The first vector</param>
        /// <param name="vec2">The second vector</param>
        /// <param name="result">The squared distance</param>
        public static void DistanceSquared(in Vector3 vec1, in Vector3 vec2, out float result)
        {
            result = (vec2.X - vec1.X) * (vec2.X - vec1.X) + (vec2.Y - vec1.Y) * (vec2.Y - vec1.Y) + (vec2.Z - vec1.Z) * (vec2.Z - vec1.Z);
        }

        /// <summary>
        /// Scale a vector to unit length
        /// </summary>
        /// <param name="vec">The input vector</param>
        /// <returns>The normalized vector</returns>
        public static Vector3 Normalize(Vector3 vec)
        {
            float scale = 1.0f / vec.Length;
            vec.X *= scale;
            vec.Y *= scale;
            vec.Z *= scale;
            return vec;
        }

        /// <summary>
        /// Scale a vector to unit length
        /// </summary>
        /// <param name="vec">The input vector</param>
        /// <param name="result">The normalized vector</param>
        public static void Normalize(in Vector3 vec, out Vector3 result)
        {
            float scale = 1.0f / vec.Length;
            result.X = vec.X * scale;
            result.Y = vec.Y * scale;
            result.Z = vec.Z * scale;
        }

        /// <summary>
        /// Scale a vector to approximately unit length
        /// </summary>
        /// <param name="vec">The input vector</param>
        /// <returns>The normalized vector</returns>
        public static Vector3 NormalizeFast(Vector3 vec)
        {
            float scale = MathDefs.InverseSqrtFast(vec.X * vec.X + vec.Y * vec.Y + vec.Z * vec.Z);
            vec.X *= scale;
            vec.Y *= scale;
            vec.Z *= scale;
            return vec;
        }

        /// <summary>
        /// Scale a vector to approximately unit length
        /// </summary>
        /// <param name="vec">The input vector</param>
        /// <param name="result">The normalized vector</param>
        public static void NormalizeFast(in Vector3 vec, out Vector3 result)
        {
            float scale = MathDefs.InverseSqrtFast(vec.X * vec.X + vec.Y * vec.Y + vec.Z * vec.Z);
            result.X = vec.X * scale;
            result.Y = vec.Y * scale;
            result.Z = vec.Z * scale;
        }

        /// <summary>
        /// Calculate the dot (scalar) product of two vectors
        /// </summary>
        /// <param name="left">First operand</param>
        /// <param name="right">Second operand</param>
        /// <returns>The dot product of the two inputs</returns>
        public static float Dot(in Vector3 left, in Vector3 right)
        {
            return left.X * right.X + left.Y * right.Y + left.Z * right.Z;
        }

        /// <summary>
        /// Calculate the dot (scalar) product of two vectors
        /// </summary>
        /// <param name="left">First operand</param>
        /// <param name="right">Second operand</param>
        /// <param name="result">The dot product of the two inputs</param>
        public static void Dot(in Vector3 left, in Vector3 right, out float result)
        {
            result = left.X * right.X + left.Y * right.Y + left.Z * right.Z;
        }

        /// <summary>
        /// Caclulate the cross (vector) product of two vectors
        /// </summary>
        /// <param name="left">First operand</param>
        /// <param name="right">Second operand</param>
        /// <returns>The cross product of the two inputs</returns>
        public static Vector3 Cross(in Vector3 left, in Vector3 right)
        {
            Vector3 result;
            Cross(left, right, out result);
            return result;
        }

        /// <summary>
        /// Caclulate the cross (vector) product of two vectors
        /// </summary>
        /// <remarks>
        /// It is incorrect to call this method passing the same variable for
        /// <paramref name="result"/> as for <paramref name="left"/> or
        /// <paramref name="right"/>.
        /// </remarks>
        /// <param name="left">First operand</param>
        /// <param name="right">Second operand</param>
        /// <returns>The cross product of the two inputs</returns>
        /// <param name="result">The cross product of the two inputs</param>
        public static void Cross(in Vector3 left, in Vector3 right, out Vector3 result)
        {
            result.X = left.Y * right.Z - left.Z * right.Y;
            result.Y = left.Z * right.X - left.X * right.Z;
            result.Z = left.X * right.Y - left.Y * right.X;
        }

        /// <summary>
        /// Returns a new Vector that is the linear blend of the 2 given Vectors
        /// </summary>
        /// <param name="a">First input vector</param>
        /// <param name="b">Second input vector</param>
        /// <param name="blend">The blend factor. a when blend=0, b when blend=1.</param>
        /// <returns>a when blend=0, b when blend=1, and a linear combination otherwise</returns>
        public static Vector3 Lerp(Vector3 a, in Vector3 b, float blend)
        {
            a.X = blend * (b.X - a.X) + a.X;
            a.Y = blend * (b.Y - a.Y) + a.Y;
            a.Z = blend * (b.Z - a.Z) + a.Z;
            return a;
        }

        /// <summary>
        /// Returns a new Vector that is the linear blend of the 2 given Vectors
        /// </summary>
        /// <param name="a">First input vector</param>
        /// <param name="b">Second input vector</param>
        /// <param name="blend">The blend factor. a when blend=0, b when blend=1.</param>
        /// <param name="result">a when blend=0, b when blend=1, and a linear combination otherwise</param>
        public static void Lerp(in Vector3 a, in Vector3 b, float blend, out Vector3 result)
        {
            result.X = blend * (b.X - a.X) + a.X;
            result.Y = blend * (b.Y - a.Y) + a.Y;
            result.Z = blend * (b.Z - a.Z) + a.Z;
        }

        /// <summary>
        /// Interpolate 3 Vectors using Barycentric coordinates
        /// </summary>
        /// <param name="a">First input Vector</param>
        /// <param name="b">Second input Vector</param>
        /// <param name="c">Third input Vector</param>
        /// <param name="u">First Barycentric Coordinate</param>
        /// <param name="v">Second Barycentric Coordinate</param>
        /// <returns>a when u=v=0, b when u=1,v=0, c when u=0,v=1, and a linear combination of a,b,c otherwise</returns>
        public static Vector3 BaryCentric(in Vector3 a, in Vector3 b, in Vector3 c, float u, float v)
        {
            return a + u * (b - a) + v * (c - a);
        }

        /// <summary>Interpolate 3 Vectors using Barycentric coordinates</summary>
        /// <param name="a">First input Vector.</param>
        /// <param name="b">Second input Vector.</param>
        /// <param name="c">Third input Vector.</param>
        /// <param name="u">First Barycentric Coordinate.</param>
        /// <param name="v">Second Barycentric Coordinate.</param>
        /// <param name="result">Output Vector. a when u=v=0, b when u=1,v=0, c when u=0,v=1, and a linear combination of a,b,c otherwise</param>
        public static void BaryCentric(in Vector3 a, in Vector3 b, in Vector3 c, float u, float v, out Vector3 result)
        {
            result = a; // copy

            Vector3 temp = b; // copy
            Subtract(temp, a, out temp);
            Multiply(temp, u, out temp);
            Add(result, temp, out result);

            temp = c; // copy
            Subtract(temp, a, out temp);
            Multiply(temp, v, out temp);
            Add(result, temp, out result);
        }

        /// <summary>Transform a direction vector by the given Matrix
        /// Assumes the matrix has a bottom row of (0,0,0,1), that is the translation part is ignored.
        /// </summary>
        /// <param name="vec">The vector to transform</param>
        /// <param name="mat">The desired transformation</param>
        /// <returns>The transformed vector</returns>
        public static Vector3 TransformVector(in Vector3 vec, in Matrix4 mat)
        {
            Vector3 result;
            TransformVector(vec, mat, out result);
            return result;
        }

        /// <summary>Transform a direction vector by the given Matrix
        /// Assumes the matrix has a bottom row of (0,0,0,1), that is the translation part is ignored.
        /// </summary>
        /// <remarks>
        /// It is incorrect to call this method passing the same variable for
        /// <paramref name="result"/> as for <paramref name="vec"/>.
        /// </remarks>
        /// <param name="vec">The vector to transform</param>
        /// <param name="mat">The desired transformation</param>
        /// <param name="result">The transformed vector</param>
        public static void TransformVector(in Vector3 vec, in Matrix4 mat, out Vector3 result)
        {
            result.X = vec.X * mat.Row0.X +
                       vec.Y * mat.Row1.X +
                       vec.Z * mat.Row2.X;

            result.Y = vec.X * mat.Row0.Y +
                       vec.Y * mat.Row1.Y +
                       vec.Z * mat.Row2.Y;

            result.Z = vec.X * mat.Row0.Z +
                       vec.Y * mat.Row1.Z +
                       vec.Z * mat.Row2.Z;
        }

        /// <summary>Transform a Normal by the given Matrix</summary>
        /// <remarks>
        /// This calculates the inverse of the given matrix, use TransformNormalInverse if you
        /// already have the inverse to avoid this extra calculation
        /// </remarks>
        /// <param name="norm">The normal to transform</param>
        /// <param name="mat">The desired transformation</param>
        /// <returns>The transformed normal</returns>
        public static Vector3 TransformNormal(in Vector3 norm, in Matrix4 mat)
        {
            Vector3 result;
            TransformNormal(norm, mat, out result);
            return result;
        }

        /// <summary>Transform a Normal by the given Matrix</summary>
        /// <remarks>
        /// This calculates the inverse of the given matrix, use TransformNormalInverse if you
        /// already have the inverse to avoid this extra calculation
        /// </remarks>
        /// <param name="norm">The normal to transform</param>
        /// <param name="mat">The desired transformation</param>
        /// <param name="result">The transformed normal</param>
        public static void TransformNormal(in Vector3 norm, in Matrix4 mat, out Vector3 result)
        {
            Matrix4 Inverse = Matrix4.Invert(mat);
            Vector3.TransformNormalInverse(norm, Inverse, out result);
        }

        /// <summary>Transform a Normal by the (transpose of the) given Matrix</summary>
        /// <remarks>
        /// This version doesn't calculate the inverse matrix.
        /// Use this version if you already have the inverse of the desired transform to hand
        /// </remarks>
        /// <param name="norm">The normal to transform</param>
        /// <param name="invMat">The inverse of the desired transformation</param>
        /// <returns>The transformed normal</returns>
        public static Vector3 TransformNormalInverse(in Vector3 norm, in Matrix4 invMat)
        {
            Vector3 result;
            TransformNormalInverse(norm, invMat, out result);
            return result;
        }

        /// <summary>Transform a Normal by the (transpose of the) given Matrix</summary>
        /// <remarks>
        /// This version doesn't calculate the inverse matrix.
        /// Use this version if you already have the inverse of the desired transform to hand
        /// </remarks>
        /// <param name="norm">The normal to transform</param>
        /// <param name="invMat">The inverse of the desired transformation</param>
        /// <param name="result">The transformed normal</param>
        public static void TransformNormalInverse(in Vector3 norm, in Matrix4 invMat, out Vector3 result)
        {
            result.X = norm.X * invMat.Row0.X +
                       norm.Y * invMat.Row0.Y +
                       norm.Z * invMat.Row0.Z;

            result.Y = norm.X * invMat.Row1.X +
                       norm.Y * invMat.Row1.Y +
                       norm.Z * invMat.Row1.Z;

            result.Z = norm.X * invMat.Row2.X +
                       norm.Y * invMat.Row2.Y +
                       norm.Z * invMat.Row2.Z;
        }

        /// <summary>Transform a Position by the given Matrix</summary>
        /// <param name="pos">The position to transform</param>
        /// <param name="mat">The desired transformation</param>
        /// <returns>The transformed position</returns>
        public static Vector3 TransformPosition(in Vector3 pos, in Matrix4 mat)
        {
            Vector3 result;
            TransformPosition(pos, mat, out result);
            return result;
        }

        /// <summary>Transform a Position by the given Matrix</summary>
        /// <param name="pos">The position to transform</param>
        /// <param name="mat">The desired transformation</param>
        /// <param name="result">The transformed position</param>
        public static void TransformPosition(in Vector3 pos, in Matrix4 mat, out Vector3 result)
        {
            result.X = pos.X * mat.Row0.X +
                       pos.Y * mat.Row1.X +
                       pos.Z * mat.Row2.X +
                       mat.Row3.X;

            result.Y = pos.X * mat.Row0.Y +
                       pos.Y * mat.Row1.Y +
                       pos.Z * mat.Row2.Y +
                       mat.Row3.Y;

            result.Z = pos.X * mat.Row0.Z +
                       pos.Y * mat.Row1.Z +
                       pos.Z * mat.Row2.Z +
                       mat.Row3.Z;
        }

        /// <summary>Transform a Vector by the given Matrix</summary>
        /// <param name="vec">The vector to transform</param>
        /// <param name="mat">The desired transformation</param>
        /// <returns>The transformed vector</returns>
        public static Vector3 Transform(in Vector3 vec, in Matrix3 mat)
        {
            Vector3 result;
            Transform(vec, mat, out result);
            return result;
        }

        /// <summary>Transform a Vector by the given Matrix</summary>
        /// <param name="vec">The vector to transform</param>
        /// <param name="mat">The desired transformation</param>
        /// <param name="result">The transformed vector</param>
        public static void Transform(in Vector3 vec, in Matrix3 mat, out Vector3 result)
        {
            result.X = vec.X * mat.Row0.X + vec.Y * mat.Row1.X + vec.Z * mat.Row2.X;
            result.Y = vec.X * mat.Row0.Y + vec.Y * mat.Row1.Y + vec.Z * mat.Row2.Y;
            result.Z = vec.X * mat.Row0.Z + vec.Y * mat.Row1.Z + vec.Z * mat.Row2.Z;
        }

        /// <summary>
        /// Transforms a vector by a quaternion rotation.
        /// </summary>
        /// <param name="vec">The vector to transform.</param>
        /// <param name="quat">The quaternion to rotate the vector by.</param>
        /// <returns>The result of the operation.</returns>
        public static Vector3 Transform(in Vector3 vec, in Quaternion quat)
        {
            Vector3 result;
            Transform(vec, quat, out result);
            return result;
        }

        /// <summary>
        /// Transforms a vector by a quaternion rotation.
        /// </summary>
        /// <param name="vec">The vector to transform.</param>
        /// <param name="quat">The quaternion to rotate the vector by.</param>
        /// <param name="result">The result of the operation.</param>
        public static void Transform(in Vector3 vec, in Quaternion quat, out Vector3 result)
        {
            // Since vec.W == 0, we can optimize quat * vec * quat^-1 as follows:
            // vec + 2.0 * cross(quat.xyz, cross(quat.xyz, vec) + quat.w * vec)
            Vector3 xyz = quat.Xyz, temp, temp2;
            Vector3.Cross(xyz, vec, out temp);
            Vector3.Multiply(vec, quat.W, out temp2);
            Vector3.Add(temp, temp2, out temp);
            Vector3.Cross(xyz, temp, out temp);
            Vector3.Multiply(temp, 2, out temp);
            Vector3.Add(vec, temp, out result);
        }

        /// <summary>Transform a Vector by the given Matrix using right-handed notation</summary>
        /// <param name="mat">The desired transformation</param>
        /// <param name="vec">The vector to transform</param>
        public static Vector3 Transform(in Matrix3 mat, in Vector3 vec)
        {
            Vector3 result;
            Transform(vec, mat, out result);
            return result;
        }

        /// <summary>Transform a Vector by the given Matrix using right-handed notation</summary>
        /// <param name="mat">The desired transformation</param>
        /// <param name="vec">The vector to transform</param>
        /// <param name="result">The transformed vector</param>
        public static void Transform(in Matrix3 mat, in Vector3 vec, out Vector3 result)
        {
            result.X = mat.Row0.X * vec.X + mat.Row0.Y * vec.Y + mat.Row0.Z * vec.Z;
            result.Y = mat.Row1.X * vec.X + mat.Row1.Y * vec.Y + mat.Row1.Z * vec.Z;
            result.Z = mat.Row2.X * vec.X + mat.Row2.Y * vec.Y + mat.Row2.Z * vec.Z;
        }

        /// <summary>Transform a Vector3 by the given Matrix, and project the resulting Vector4 back to a Vector3</summary>
        /// <param name="vec">The vector to transform</param>
        /// <param name="mat">The desired transformation</param>
        /// <returns>The transformed vector</returns>
        public static Vector3 TransformPerspective(in Vector3 vec, in Matrix4 mat)
        {
            Vector3 result;
            TransformPerspective(vec, mat, out result);
            return result;
        }

        /// <summary>Transform a Vector3 by the given Matrix, and project the resulting Vector4 back to a Vector3</summary>
        /// <param name="vec">The vector to transform</param>
        /// <param name="mat">The desired transformation</param>
        /// <param name="result">The transformed vector</param>
        public static void TransformPerspective(in Vector3 vec, in Matrix4 mat, out Vector3 result)
        {
            Vector4 v = new Vector4(vec.X, vec.Y, vec.Z, 1);
            Vector4.Transform(v, mat, out v);
            result.X = v.X / v.W;
            result.Y = v.Y / v.W;
            result.Z = v.Z / v.W;
        }

        /// <summary>
        /// Calculates the angle (in radians) between two vectors.
        /// </summary>
        /// <param name="first">The first vector.</param>
        /// <param name="second">The second vector.</param>
        /// <returns>Angle (in radians) between the vectors.</returns>
        /// <remarks>Note that the returned angle is never bigger than the constant Pi.</remarks>
        public static float CalculateAngle(in Vector3 first, in Vector3 second)
        {
            float result;
            CalculateAngle(first, second, out result);
            return result;
        }

        /// <summary>Calculates the angle (in radians) between two vectors.</summary>
        /// <param name="first">The first vector.</param>
        /// <param name="second">The second vector.</param>
        /// <param name="result">Angle (in radians) between the vectors.</param>
        /// <remarks>Note that the returned angle is never bigger than the constant Pi.</remarks>
        public static void CalculateAngle(in Vector3 first, in Vector3 second, out float result)
        {
            float temp;
            Vector3.Dot(first, second, out temp);
            result = (float)System.Math.Acos(MathDefs.Clamp(temp / (first.Length * second.Length), -1.0, 1.0));
        }

        /// <summary>
        /// Projects a vector from object space into screen space.
        /// </summary>
        /// <param name="vector">The vector to project.</param>
        /// <param name="x">The X coordinate of the viewport.</param>
        /// <param name="y">The Y coordinate of the viewport.</param>
        /// <param name="width">The width of the viewport.</param>
        /// <param name="height">The height of the viewport.</param>
        /// <param name="minZ">The minimum depth of the viewport.</param>
        /// <param name="maxZ">The maximum depth of the viewport.</param>
        /// <param name="worldViewProjection">The world-view-projection matrix.</param>
        /// <returns>The vector in screen space.</returns>
        /// <remarks>
        /// To project to normalized device coordinates (NDC) use the following parameters:
        /// Project(vector, -1, -1, 2, 2, -1, 1, worldViewProjection).
        /// </remarks>
        public static Vector3 Project(in Vector3 vector, float x, float y, float width, float height, float minZ, float maxZ, in Matrix4 worldViewProjection)
        {
            Vector4 result;

            result.X =
                vector.X * worldViewProjection.M00 +
                vector.Y * worldViewProjection.M10 +
                vector.Z * worldViewProjection.M20 +
                worldViewProjection.M30;

            result.Y =
                vector.X * worldViewProjection.M01 +
                vector.Y * worldViewProjection.M11 +
                vector.Z * worldViewProjection.M21 +
                worldViewProjection.M20;

            result.Z =
                vector.X * worldViewProjection.M02 +
                vector.Y * worldViewProjection.M12 +
                vector.Z * worldViewProjection.M11 +
                worldViewProjection.M21;

            result.W =
                vector.X * worldViewProjection.M03 +
                vector.Y * worldViewProjection.M13 +
                vector.Z * worldViewProjection.M12 +
                worldViewProjection.M11;

            result /= result.W;

            result.X = x + (width * ((result.X + 1.0f) / 2.0f));
            result.Y = y + (height * ((result.Y + 1.0f) / 2.0f));
            result.Z = minZ + ((maxZ - minZ) * ((result.Z + 1.0f) / 2.0f));

            return new Vector3(result.X, result.Y, result.Z);
        }

        /// <summary>
        /// Projects a vector from screen space into object space.
        /// </summary>
        /// <param name="vector">The vector to project.</param>
        /// <param name="x">The X coordinate of the viewport.</param>
        /// <param name="y">The Y coordinate of the viewport.</param>
        /// <param name="width">The width of the viewport.</param>
        /// <param name="height">The height of the viewport.</param>
        /// <param name="minZ">The minimum depth of the viewport.</param>
        /// <param name="maxZ">The maximum depth of the viewport.</param>
        /// <param name="inverseWorldViewProjection">The inverse of the world-view-projection matrix.</param>
        /// <returns>The vector in object space.</returns>
        /// <remarks>
        /// To project from normalized device coordinates (NDC) use the following parameters:
        /// Project(vector, -1, -1, 2, 2, -1, 1, inverseWorldViewProjection).
        /// </remarks>
        public static Vector3 Unproject(in Vector3 vector, float x, float y, float width, float height, float minZ, float maxZ, in Matrix4 inverseWorldViewProjection)
        {
            Vector4 result;

            result.X = ((((vector.X - x) / width) * 2.0f) - 1.0f);
            result.Y = ((((vector.Y - y) / height) * 2.0f) - 1.0f);
            result.Z = (((vector.Z / (maxZ - minZ)) * 2.0f) - 1.0f);

            result.X =
                result.X * inverseWorldViewProjection.M00 +
                result.Y * inverseWorldViewProjection.M10 +
                result.Z * inverseWorldViewProjection.M20 +
                inverseWorldViewProjection.M30;

            result.Y =
                result.X * inverseWorldViewProjection.M01 +
                result.Y * inverseWorldViewProjection.M11 +
                result.Z * inverseWorldViewProjection.M21 +
                inverseWorldViewProjection.M20;

            result.Z =
                result.X * inverseWorldViewProjection.M02 +
                result.Y * inverseWorldViewProjection.M12 +
                result.Z * inverseWorldViewProjection.M11 +
                inverseWorldViewProjection.M21;

            result.W =
                result.X * inverseWorldViewProjection.M03 +
                result.Y * inverseWorldViewProjection.M13 +
                result.Z * inverseWorldViewProjection.M12 +
                inverseWorldViewProjection.M11;

            result /= result.W;

            return new Vector3(result.X, result.Y, result.Z);
        }

        /// <summary>
        /// Gets or sets an OpenTK.Vector2 with the X and Y components of this instance.
        /// </summary>
        [XmlIgnore]
        public Vector2 Xy { get { return new Vector2(X, Y); } set { X = value.X; Y = value.Y; } }

        /// <summary>
        /// Gets or sets an OpenTK.Vector2 with the X and Z components of this instance.
        /// </summary>
        [XmlIgnore]
        public Vector2 Xz { get { return new Vector2(X, Z); } set { X = value.X; Z = value.Y; } }

        /// <summary>
        /// Gets or sets an OpenTK.Vector2 with the Y and X components of this instance.
        /// </summary>
        [XmlIgnore]
        public Vector2 Yx { get { return new Vector2(Y, X); } set { Y = value.X; X = value.Y; } }

        /// <summary>
        /// Gets or sets an OpenTK.Vector2 with the Y and Z components of this instance.
        /// </summary>
        [XmlIgnore]
        public Vector2 Yz { get { return new Vector2(Y, Z); } set { Y = value.X; Z = value.Y; } }

        /// <summary>
        /// Gets or sets an OpenTK.Vector2 with the Z and X components of this instance.
        /// </summary>
        [XmlIgnore]
        public Vector2 Zx { get { return new Vector2(Z, X); } set { Z = value.X; X = value.Y; } }

        /// <summary>
        /// Gets or sets an OpenTK.Vector2 with the Z and Y components of this instance.
        /// </summary>
        [XmlIgnore]
        public Vector2 Zy { get { return new Vector2(Z, Y); } set { Z = value.X; Y = value.Y; } }

        /// <summary>
        /// Gets or sets an OpenTK.Vector3 with the X, Z, and Y components of this instance.
        /// </summary>
        [XmlIgnore]
        public Vector3 Xzy { get { return new Vector3(X, Z, Y); } set { X = value.X; Z = value.Y; Y = value.Z; } }

        /// <summary>
        /// Gets or sets an OpenTK.Vector3 with the Y, X, and Z components of this instance.
        /// </summary>
        [XmlIgnore]
        public Vector3 Yxz { get { return new Vector3(Y, X, Z); } set { Y = value.X; X = value.Y; Z = value.Z; } }

        /// <summary>
        /// Gets or sets an OpenTK.Vector3 with the Y, Z, and X components of this instance.
        /// </summary>
        [XmlIgnore]
        public Vector3 Yzx { get { return new Vector3(Y, Z, X); } set { Y = value.X; Z = value.Y; X = value.Z; } }

        /// <summary>
        /// Gets or sets an OpenTK.Vector3 with the Z, X, and Y components of this instance.
        /// </summary>
        [XmlIgnore]
        public Vector3 Zxy { get { return new Vector3(Z, X, Y); } set { Z = value.X; X = value.Y; Y = value.Z; } }

        /// <summary>
        /// Gets or sets an OpenTK.Vector3 with the Z, Y, and X components of this instance.
        /// </summary>
        [XmlIgnore]
        public Vector3 Zyx { get { return new Vector3(Z, Y, X); } set { Z = value.X; Y = value.Y; X = value.Z; } }

        /// <summary>
        /// Adds two instances.
        /// </summary>
        /// <param name="left">The first instance.</param>
        /// <param name="right">The second instance.</param>
        /// <returns>The result of the calculation.</returns>
        public static Vector3 operator +(Vector3 left, in Vector3 right)
        {
            left.X += right.X;
            left.Y += right.Y;
            left.Z += right.Z;
            return left;
        }

        /// <summary>
        /// Subtracts two instances.
        /// </summary>
        /// <param name="left">The first instance.</param>
        /// <param name="right">The second instance.</param>
        /// <returns>The result of the calculation.</returns>
        public static Vector3 operator -(Vector3 left, in Vector3 right)
        {
            left.X -= right.X;
            left.Y -= right.Y;
            left.Z -= right.Z;
            return left;
        }

        /// <summary>
        /// Negates an instance.
        /// </summary>
        /// <param name="vec">The instance.</param>
        /// <returns>The result of the calculation.</returns>
        public static Vector3 operator -(Vector3 vec)
        {
            vec.X = -vec.X;
            vec.Y = -vec.Y;
            vec.Z = -vec.Z;
            return vec;
        }

        /// <summary>
        /// Multiplies an instance by a scalar.
        /// </summary>
        /// <param name="vec">The instance.</param>
        /// <param name="scale">The scalar.</param>
        /// <returns>The result of the calculation.</returns>
        public static Vector3 operator *(Vector3 vec, float scale)
        {
            vec.X *= scale;
            vec.Y *= scale;
            vec.Z *= scale;
            return vec;
        }

        /// <summary>
        /// Multiplies an instance by a scalar.
        /// </summary>
        /// <param name="scale">The scalar.</param>
        /// <param name="vec">The instance.</param>
        /// <returns>The result of the calculation.</returns>
        public static Vector3 operator *(float scale, Vector3 vec)
        {
            vec.X *= scale;
            vec.Y *= scale;
            vec.Z *= scale;
            return vec;
        }

        /// <summary>
        /// Component-wise multiplication between the specified instance by a scale vector.
        /// </summary>
        /// <param name="scale">Left operand.</param>
        /// <param name="vec">Right operand.</param>
        /// <returns>Result of multiplication.</returns>
        public static Vector3 operator *(Vector3 vec, in Vector3 scale)
        {
            vec.X *= scale.X;
            vec.Y *= scale.Y;
            vec.Z *= scale.Z;
            return vec;
        }

        /// <summary>
        /// Transform a Vector by the given Matrix.
        /// </summary>
        /// <param name="vec">The vector to transform</param>
        /// <param name="mat">The desired transformation</param>
        /// <returns>The transformed vector</returns>
        public static Vector3 operator *(in Vector3 vec, in Matrix3 mat)
        {
            Vector3 result;
            Vector3.Transform(vec, mat, out result);
            return result;
        }

        /// <summary>
        /// Transform a Vector by the given Matrix using right-handed notation
        /// </summary>
        /// <param name="mat">The desired transformation</param>
        /// <param name="vec">The vector to transform</param>
        /// <returns>The transformed vector</returns>
        public static Vector3 operator *(in Matrix3 mat, in Vector3 vec)
        {
            Vector3 result;
            Vector3.Transform(mat, vec, out result);
            return result;
        }

        /// <summary>
        /// Transforms a vector by a quaternion rotation.
        /// </summary>
        /// <param name="vec">The vector to transform.</param>
        /// <param name="quat">The quaternion to rotate the vector by.</param>
        /// <returns></returns>
        public static Vector3 operator *(in Quaternion quat, in Vector3 vec)
        {
            Vector3 result;
            Vector3.Transform(vec, quat, out result);
            return result;
        }

        /// <summary>
        /// Divides an instance by a scalar.
        /// </summary>
        /// <param name="vec">The instance.</param>
        /// <param name="scale">The scalar.</param>
        /// <returns>The result of the calculation.</returns>
        public static Vector3 operator /(Vector3 vec, float scale)
        {
            vec.X /= scale;
            vec.Y /= scale;
            vec.Z /= scale;
            return vec;
        }

        /// <summary>
        /// Compares two instances for equality.
        /// </summary>
        /// <param name="left">The first instance.</param>
        /// <param name="right">The second instance.</param>
        /// <returns>True, if left equals right; false otherwise.</returns>
        public static bool operator ==(in Vector3 left, in Vector3 right)
        {
            return left.Equals(right);
        }

        /// <summary>
        /// Compares two instances for inequality.
        /// </summary>
        /// <param name="left">The first instance.</param>
        /// <param name="right">The second instance.</param>
        /// <returns>True, if left does not equal right; false otherwise.</returns>
        public static bool operator !=(in Vector3 left, in Vector3 right)
        {
            return !left.Equals(right);
        }

        private static string listSeparator = System.Globalization.CultureInfo.CurrentCulture.TextInfo.ListSeparator;
        /// <summary>
        /// Returns a System.String that represents the current Vector3.
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return String.Format("({0}{3} {1}{3} {2})", X, Y, Z, listSeparator);
        }

        /// <summary>
        /// Returns the hashcode for this instance.
        /// </summary>
        /// <returns>A System.Int32 containing the unique hashcode for this instance.</returns>
        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = this.X.GetHashCode();
                hashCode = (hashCode * 397) ^ this.Y.GetHashCode();
                hashCode = (hashCode * 397) ^ this.Z.GetHashCode();
                return hashCode;
            }
        }

        /// <summary>
        /// Indicates whether this instance and a specified object are equal.
        /// </summary>
        /// <param name="obj">The object to compare to.</param>
        /// <returns>True if the instances are equal; false otherwise.</returns>
        public override bool Equals(object obj)
        {
            if (!(obj is Vector3))
            {
                return false;
            }

            return this.Equals((Vector3)obj);
        }

        /// <summary>Indicates whether the current vector is equal to another vector.</summary>
        /// <param name="other">A vector to compare with this vector.</param>
        /// <returns>true if the current vector is equal to the vector parameter; otherwise, false.</returns>
        public bool Equals(Vector3 other)
        {
            return
                X == other.X &&
                Y == other.Y &&
                Z == other.Z;
        }

        /// Calculate dot product.
        public float DotProduct(in Vector3 rhs) { return X * rhs.X + Y * rhs.Y + Z * rhs.Z; }

        /// Calculate absolute dot product.
        public float AbsDotProduct(in Vector3 rhs)
        {
            return Math.Abs(X * rhs.X) + Math.Abs(Y * rhs.Y) + Math.Abs(Z * rhs.Z);
        }

        /// Project direction vector onto axis.
        public float ProjectOntoAxis(in Vector3 axis) { return DotProduct(axis.Normalized()); }

        /// Project position vector onto plane with given origin and normal.
        public Vector3 ProjectOntoPlane(in Vector3 origin, in Vector3 normal)
        {
            Vector3 delta = this - origin;
            return this - normal.Normalized() * delta.ProjectOntoAxis(normal);
        }

        /// Project position vector onto line segment.
        public Vector3 ProjectOntoLine(in Vector3 from, in Vector3 to, bool clamped = false)
        {
            Vector3 direction = to - from;
            float lengthSquared = direction.LengthSquared;
            float factor = (this - from).DotProduct(direction) / lengthSquared;

            if (clamped)
                factor = MathDefs.Clamp(factor, 0.0f, 1.0f);

            return from + direction * factor;
        }

        /// Calculate distance to another position vector.
        public float DistanceToPoint(in Vector3 point) { return (this - point).Length; }

        /// Calculate distance to the plane with given origin and normal.
        public float DistanceToPlane(in Vector3 origin, in Vector3 normal) { return (this - origin).ProjectOntoAxis(normal); }

        /// Make vector orthogonal to the axis.
        public Vector3 Orthogonalize(in Vector3 axis) { return axis.CrossProduct(this).CrossProduct(axis).Normalized(); }

        /// Calculate cross product.
        public Vector3 CrossProduct(in Vector3 rhs)
        {
            return new Vector3(
                Y * rhs.Z - Z * rhs.Y,
                Z * rhs.X - X * rhs.Z,
                X * rhs.Y - Y * rhs.X
            );
        }
        /// Return absolute vector.
        public Vector3 Abs() { return new Vector3(Math.Abs(X), Math.Abs(Y), Math.Abs(Z)); }
    }
}
