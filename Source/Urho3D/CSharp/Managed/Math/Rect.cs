using System;
using System.Runtime.InteropServices;

namespace Urho3DNet
{
    [StructLayout(LayoutKind.Sequential)]
    public struct Rect
    {
        /// Minimum vector.
        public Vector2 Min;
        /// Maximum vector.
        public Vector2 Max;

        public Rect(float minX, float minY, float maxX, float maxY)
        {
            Min = new Vector2(minX, minY);
            Max = new Vector2(maxX, maxY);
        }

        public Rect(in Vector2 min, in Vector2 max)
        {
            Min = min;
            Max = max;
        }

        /// Merge a point.
        public void Merge(in Vector2 point)
        {
            if (point.X < Min.X)
                Min.X = point.X;
            if (point.X > Max.X)
                Max.X = point.X;
            if (point.Y < Min.Y)
                Min.Y = point.Y;
            if (point.Y > Max.Y)
                Max.Y = point.Y;
        }

        /// Merge a rect.
        public void Merge(in Rect rect)
        {
            if (rect.Min.X < Min.X)
                Min.X = rect.Min.X;
            if (rect.Min.Y < Min.Y)
                Min.Y = rect.Min.Y;
            if (rect.Max.X > Max.X)
                Max.X = rect.Max.X;
            if (rect.Max.Y > Max.Y)
                Max.Y = rect.Max.Y;
        }

        public static readonly Rect Full = new Rect(-1.0f, -1.0f, 1.0f, 1.0f);
        public static readonly Rect Positive = new Rect(0.0f, 0.0f, 1.0f, 1.0f);
        public static readonly Rect Zero = new Rect(0.0f, 0.0f, 0.0f, 0.0f);
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct IntRect : IEquatable<IntRect>
    {
        public bool Equals(IntRect other)
        {
            return Left == other.Left && Top == other.Top && Right == other.Right && Bottom == other.Bottom;
        }

        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            return obj is IntRect && Equals((IntRect) obj);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = Left;
                hashCode = (hashCode * 397) ^ Top;
                hashCode = (hashCode * 397) ^ Right;
                hashCode = (hashCode * 397) ^ Bottom;
                return hashCode;
            }
        }

        /// Left coordinate.
        public int Left;
        /// Top coordinate.
        public int Top;
        /// Right coordinate.
        public int Right;
        /// Bottom coordinate.
        public int Bottom;

        public IntRect(int left, int top, int right, int bottom)
        {
            Left = left;
            Top = top;
            Right = right;
            Bottom = bottom;
        }

        public static readonly IntRect Zero = new IntRect(0, 0, 0, 0);

        public IntRect(in IntVector2 min, in IntVector2 max)
        {
            Left = min.X;
            Top = min.Y;
            Right = max.X;
            Bottom = max.Y;
        }

        public int Width => Right - Left;
        public int Height => Bottom - Top;
        public IntVector2 Size => new IntVector2(Width, Height);

        public static bool operator !=(in IntRect lhs, in IntRect rhs)
        {
            return lhs.Top != rhs.Top || lhs.Right != rhs.Right || lhs.Bottom != rhs.Bottom || lhs.Left != rhs.Left;
        }

        public static bool operator ==(in IntRect lhs, in IntRect rhs)
        {
            return !(lhs != rhs);
        }
    }
}
