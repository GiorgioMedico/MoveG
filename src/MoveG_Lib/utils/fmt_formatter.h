#pragma once

#include <Eigen/Dense>
#include <fmt/core.h>
#include <fmt/format.h>

namespace MoveG
{

//------------------------------------------------------------------------------
// Vector formatter (for Vector2d, Vector3d, Vector4d, etc.)
//------------------------------------------------------------------------------
template <typename Scalar, int Dim>
struct formatter<Eigen::Matrix<Scalar, Dim, 1>>
{
    // Format spec parser
    constexpr auto parse(format_parse_context &ctx) -> decltype(ctx.begin())
    {
        auto it = ctx.begin(), end = ctx.end();
        if (it != end && *it != '}')
        {
            // Parse format specifications
            it = std::find(it, end, '}');
        }
        return it;
    }

    // Formatter
    template <typename FormatContext>
    auto format(const Eigen::Matrix<Scalar, Dim, 1> &v, FormatContext &ctx) const
        -> decltype(ctx.out())
    {
        auto out = format_to(ctx.out(), "[");

        for (int i = 0; i < v.size(); ++i)
        {
            if (i > 0)
            {
                out = format_to(out, ", ");
            }
            out = format_to(out, "{:.6g}", v[i]);
        }

        return format_to(out, "]");
    }
};

//------------------------------------------------------------------------------
// Matrix formatter
//------------------------------------------------------------------------------
template <typename Scalar, int Rows, int Cols>
struct formatter<Eigen::Matrix<Scalar, Rows, Cols>>
{
    // Skip this specialization for vectors which are already handled
    static_assert(Cols != 1, "Use the Vector formatter for column vectors");

    constexpr auto parse(format_parse_context &ctx) -> decltype(ctx.begin())
    {
        auto it = ctx.begin(), end = ctx.end();
        if (it != end && *it != '}')
        {
            it = std::find(it, end, '}');
        }
        return it;
    }

    template <typename FormatContext>
    auto format(const Eigen::Matrix<Scalar, Rows, Cols> &matrix, FormatContext &ctx) const
        -> decltype(ctx.out())
    {
        auto out = ctx.out();

        // Format as a multi-line matrix
        for (int i = 0; i < matrix.rows(); ++i)
        {
            if (i == 0)
            {
                out = format_to(out, "⎡");
            }
            else if (i == matrix.rows() - 1)
            {
                out = format_to(out, "⎣");
            }
            else
            {
                out = format_to(out, "⎢");
            }

            for (int j = 0; j < matrix.cols(); ++j)
            {
                if (j > 0)
                {
                    out = format_to(out, " ");
                }
                out = format_to(out, "{:8.4f}", matrix(i, j));
            }

            if (i == 0)
            {
                out = format_to(out, "⎤\n");
            }
            else if (i == matrix.rows() - 1)
            {
                out = format_to(out, "⎦");
            }
            else
            {
                out = format_to(out, "⎥\n");
            }
        }

        return out;
    }
};

//------------------------------------------------------------------------------
// Quaternion formatter
//------------------------------------------------------------------------------
template <typename Scalar>
struct formatter<Eigen::Quaternion<Scalar>>
{
    constexpr auto parse(format_parse_context &ctx) -> decltype(ctx.begin())
    {
        auto it = ctx.begin(), end = ctx.end();
        if (it != end && *it != '}')
        {
            it = std::find(it, end, '}');
        }
        return it;
    }

    template <typename FormatContext>
    auto format(const Eigen::Quaternion<Scalar> &q, FormatContext &ctx) const -> decltype(ctx.out())
    {
        return format_to(ctx.out(),
                         "[w={:.6g}, x={:.6g}, y={:.6g}, z={:.6g}]",
                         q.w(),
                         q.x(),
                         q.y(),
                         q.z());
    }
};

//------------------------------------------------------------------------------
// Affine transformation formatter
//------------------------------------------------------------------------------
template <typename Scalar, int Dim>
struct formatter<Eigen::Transform<Scalar, Dim, Eigen::Affine>>
{
    constexpr auto parse(format_parse_context &ctx) -> decltype(ctx.begin())
    {
        auto it = ctx.begin(), end = ctx.end();
        if (it != end && *it != '}')
        {
            it = std::find(it, end, '}');
        }
        return it;
    }

    template <typename FormatContext>
    auto format(const Eigen::Transform<Scalar, Dim, Eigen::Affine> &t, FormatContext &ctx) const
        -> decltype(ctx.out())
    {
        // For Affine transformations, we'll format as a homogeneous matrix
        auto matrix = t.matrix();
        return formatter<Eigen::Matrix<Scalar, Dim + 1, Dim + 1>>().format(matrix, ctx);
    }
};

//------------------------------------------------------------------------------
// AngleAxis formatter
//------------------------------------------------------------------------------
template <typename Scalar>
struct formatter<Eigen::AngleAxis<Scalar>>
{
    constexpr auto parse(format_parse_context &ctx) -> decltype(ctx.begin())
    {
        auto it = ctx.begin(), end = ctx.end();
        if (it != end && *it != '}')
        {
            it = std::find(it, end, '}');
        }
        return it;
    }

    template <typename FormatContext>
    auto format(const Eigen::AngleAxis<Scalar> &aa, FormatContext &ctx) const -> decltype(ctx.out())
    {
        return format_to(ctx.out(),
                         "angle={:.4f}° axis=[{:.6g}, {:.6g}, {:.6g}]",
                         aa.angle() * 180.0 / M_PI,
                         aa.axis().x(),
                         aa.axis().y(),
                         aa.axis().z());
    }
};

} // namespace MoveG
