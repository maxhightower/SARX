#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

#include "sarx/gltf_character.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace sarx {
namespace {

struct Quat {
    double x{};
    double y{};
    double z{};
    double w{1.0};
};

struct Mat4 {
    std::array<double, 16> m{};
};

Mat4 identity() {
    Mat4 out;
    out.m[0] = 1.0;
    out.m[5] = 1.0;
    out.m[10] = 1.0;
    out.m[15] = 1.0;
    return out;
}

Mat4 multiply(const Mat4& a, const Mat4& b) {
    Mat4 out;
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            double value = 0.0;
            for (int k = 0; k < 4; ++k) {
                value +=
                    a.m[k * 4 + row]
                    * b.m[col * 4 + k];
            }
            out.m[col * 4 + row] = value;
        }
    }
    return out;
}

Vec3 transform_point(const Mat4& m, const Vec3& p) {
    const double x =
        m.m[0] * p.x
        + m.m[4] * p.y
        + m.m[8] * p.z
        + m.m[12];
    const double y =
        m.m[1] * p.x
        + m.m[5] * p.y
        + m.m[9] * p.z
        + m.m[13];
    const double z =
        m.m[2] * p.x
        + m.m[6] * p.y
        + m.m[10] * p.z
        + m.m[14];
    const double w =
        m.m[3] * p.x
        + m.m[7] * p.y
        + m.m[11] * p.z
        + m.m[15];

    if (std::abs(w) > 1e-12 && std::abs(w - 1.0) > 1e-12) {
        return {x / w, y / w, z / w};
    }
    return {x, y, z};
}

Mat4 inverse_affine(const Mat4& matrix) {
    const double a00 = matrix.m[0];
    const double a01 = matrix.m[4];
    const double a02 = matrix.m[8];
    const double a10 = matrix.m[1];
    const double a11 = matrix.m[5];
    const double a12 = matrix.m[9];
    const double a20 = matrix.m[2];
    const double a21 = matrix.m[6];
    const double a22 = matrix.m[10];

    const double determinant =
        a00 * (a11 * a22 - a12 * a21)
        - a01 * (a10 * a22 - a12 * a20)
        + a02 * (a10 * a21 - a11 * a20);

    if (std::abs(determinant) <= 1e-12) {
        throw std::runtime_error(
            "cannot invert singular affine transform");
    }

    const double inv_det = 1.0 / determinant;

    Mat4 inverse = identity();

    inverse.m[0] =
        (a11 * a22 - a12 * a21) * inv_det;
    inverse.m[4] =
        (a02 * a21 - a01 * a22) * inv_det;
    inverse.m[8] =
        (a01 * a12 - a02 * a11) * inv_det;

    inverse.m[1] =
        (a12 * a20 - a10 * a22) * inv_det;
    inverse.m[5] =
        (a00 * a22 - a02 * a20) * inv_det;
    inverse.m[9] =
        (a02 * a10 - a00 * a12) * inv_det;

    inverse.m[2] =
        (a10 * a21 - a11 * a20) * inv_det;
    inverse.m[6] =
        (a01 * a20 - a00 * a21) * inv_det;
    inverse.m[10] =
        (a00 * a11 - a01 * a10) * inv_det;

    const Vec3 translation{
        matrix.m[12],
        matrix.m[13],
        matrix.m[14]
    };

    const Vec3 inverse_translation =
        transform_point(
            inverse,
            -translation);

    inverse.m[12] = inverse_translation.x;
    inverse.m[13] = inverse_translation.y;
    inverse.m[14] = inverse_translation.z;

    return inverse;
}

Quat normalized_quat(Quat q) {
    const double len = std::sqrt(
        q.x * q.x
        + q.y * q.y
        + q.z * q.z
        + q.w * q.w);

    if (len <= 1e-12) {
        return {};
    }

    q.x /= len;
    q.y /= len;
    q.z /= len;
    q.w /= len;
    return q;
}

Quat slerp(Quat a, Quat b, double t) {
    a = normalized_quat(a);
    b = normalized_quat(b);

    double d =
        a.x * b.x
        + a.y * b.y
        + a.z * b.z
        + a.w * b.w;

    if (d < 0.0) {
        d = -d;
        b.x = -b.x;
        b.y = -b.y;
        b.z = -b.z;
        b.w = -b.w;
    }

    if (d > 0.9995) {
        return normalized_quat({
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t,
            a.w + (b.w - a.w) * t
        });
    }

    const double theta0 = std::acos(
        std::clamp(d, -1.0, 1.0));
    const double theta = theta0 * t;
    const double sin_theta = std::sin(theta);
    const double sin_theta0 = std::sin(theta0);

    const double s0 =
        std::cos(theta)
        - d * sin_theta / sin_theta0;
    const double s1 =
        sin_theta / sin_theta0;

    return {
        s0 * a.x + s1 * b.x,
        s0 * a.y + s1 * b.y,
        s0 * a.z + s1 * b.z,
        s0 * a.w + s1 * b.w
    };
}

Mat4 trs(
    const Vec3& translation,
    Quat rotation,
    const Vec3& scale) {

    rotation = normalized_quat(rotation);

    const double x = rotation.x;
    const double y = rotation.y;
    const double z = rotation.z;
    const double w = rotation.w;

    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;
    const double xy = x * y;
    const double xz = x * z;
    const double yz = y * z;
    const double xw = x * w;
    const double yw = y * w;
    const double zw = z * w;

    Mat4 out = identity();

    out.m[0] = (1.0 - 2.0 * (yy + zz)) * scale.x;
    out.m[1] = (2.0 * (xy + zw)) * scale.x;
    out.m[2] = (2.0 * (xz - yw)) * scale.x;

    out.m[4] = (2.0 * (xy - zw)) * scale.y;
    out.m[5] = (1.0 - 2.0 * (xx + zz)) * scale.y;
    out.m[6] = (2.0 * (yz + xw)) * scale.y;

    out.m[8] = (2.0 * (xz + yw)) * scale.z;
    out.m[9] = (2.0 * (yz - xw)) * scale.z;
    out.m[10] = (1.0 - 2.0 * (xx + yy)) * scale.z;

    out.m[12] = translation.x;
    out.m[13] = translation.y;
    out.m[14] = translation.z;

    return out;
}

std::string lower_copy(std::string value) {
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
    return value;
}

struct CgltfOwner {
    cgltf_data* data{nullptr};

    ~CgltfOwner() {
        if (data) {
            cgltf_free(data);
        }
    }

    CgltfOwner() = default;
    CgltfOwner(const CgltfOwner&) = delete;
    CgltfOwner& operator=(const CgltfOwner&) = delete;

    CgltfOwner(CgltfOwner&& other) noexcept
        : data(other.data) {
        other.data = nullptr;
    }

    CgltfOwner& operator=(CgltfOwner&& other) noexcept {
        if (this == &other) return *this;
        if (data) cgltf_free(data);
        data = other.data;
        other.data = nullptr;
        return *this;
    }
};

CgltfOwner load_gltf(const std::string& path) {
    cgltf_options options{};
    CgltfOwner owner;

    const cgltf_result parsed =
        cgltf_parse_file(
            &options,
            path.c_str(),
            &owner.data);

    if (parsed != cgltf_result_success || !owner.data) {
        throw std::runtime_error(
            "failed to parse glTF: " + path);
    }

    const cgltf_result buffers =
        cgltf_load_buffers(
            &options,
            owner.data,
            path.c_str());

    if (buffers != cgltf_result_success) {
        throw std::runtime_error(
            "failed to load glTF buffers: " + path);
    }

    return owner;
}

int node_index(
    const cgltf_data* data,
    const cgltf_node* node) {

    if (!node) return -1;
    return static_cast<int>(node - data->nodes);
}

int skin_index(
    const cgltf_data* data,
    const cgltf_skin* skin) {

    if (!skin) return -1;
    return static_cast<int>(skin - data->skins);
}

struct NodePose {
    int parent{-1};

    bool matrix_mode{false};
    Mat4 matrix{};

    Vec3 translation{};
    Quat rotation{};
    Vec3 scale{1.0, 1.0, 1.0};

    std::string name;
};

struct SkinData {
    std::vector<int> joints;
    std::vector<Mat4> inverse_bind;
};

struct VertexData {
    Vec3 position{};
    std::array<std::uint32_t, 4> joints{};
    std::array<double, 4> weights{};
    bool skinned{false};
};

struct MeshPart {
    int node{-1};
    int skin{-1};
    std::vector<VertexData> vertices;
    std::vector<std::uint32_t> indices;
};

enum class TrackPath {
    Translation,
    Rotation,
    Scale
};

enum class TrackInterpolation {
    Linear,
    Step,
    Cubic
};

struct Track {
    int node{-1};
    TrackPath path{TrackPath::Translation};
    TrackInterpolation interpolation{TrackInterpolation::Linear};
    std::vector<double> times;
    std::vector<std::array<double, 4>> values;
};

struct Clip {
    std::string name;
    double duration{};
    std::vector<Track> tracks;
};

std::array<double, 4> read_value(
    const cgltf_accessor* accessor,
    cgltf_size index,
    cgltf_size components) {

    cgltf_float temp[4]{};
    if (!cgltf_accessor_read_float(
            accessor,
            index,
            temp,
            components)) {
        throw std::runtime_error(
            "failed to read glTF animation value");
    }

    return {
        static_cast<double>(temp[0]),
        static_cast<double>(temp[1]),
        static_cast<double>(temp[2]),
        static_cast<double>(temp[3])
    };
}

std::array<double, 4> sample_track(
    const Track& track,
    double time) {

    if (track.times.empty() || track.values.empty()) {
        return {};
    }

    if (time <= track.times.front()) {
        return track.values.front();
    }

    if (time >= track.times.back()) {
        return track.values.back();
    }

    const auto upper =
        std::upper_bound(
            track.times.begin(),
            track.times.end(),
            time);

    const std::size_t i1 =
        static_cast<std::size_t>(
            upper - track.times.begin());
    const std::size_t i0 = i1 - 1;

    if (track.interpolation == TrackInterpolation::Step) {
        return track.values[i0];
    }

    const double t0 = track.times[i0];
    const double t1 = track.times[i1];
    const double alpha =
        (t1 - t0) > 1e-12
        ? (time - t0) / (t1 - t0)
        : 0.0;

    if (track.path == TrackPath::Rotation) {
        const auto& a = track.values[i0];
        const auto& b = track.values[i1];

        const Quat q = slerp(
            {a[0], a[1], a[2], a[3]},
            {b[0], b[1], b[2], b[3]},
            alpha);

        return {q.x, q.y, q.z, q.w};
    }

    std::array<double, 4> out{};
    for (std::size_t component = 0;
         component < 4;
         ++component) {
        out[component] =
            track.values[i0][component]
            + (track.values[i1][component]
               - track.values[i0][component])
                * alpha;
    }
    return out;
}

} // namespace

struct GltfCharacter::Impl {
    CharacterAssetStats stats{};
    std::vector<std::string> animation_names;
    std::vector<CharacterJointInfo> joint_infos;

    std::vector<NodePose> rest_nodes;
    std::vector<SkinData> skins;
    std::vector<MeshPart> parts;
    std::vector<Clip> clips;

    std::unordered_map<std::string, int> node_by_name;
};

namespace {

std::vector<Mat4> sampled_globals(
    const std::vector<NodePose>& rest_nodes,
    const Clip& clip,
    double time_seconds,
    bool loop) {

    double time = time_seconds;

    if (clip.duration > 1e-12) {
        if (loop) {
            time = std::fmod(
                std::max(0.0, time),
                clip.duration);
        } else {
            time = std::clamp(
                time,
                0.0,
                clip.duration);
        }
    }

    std::vector<NodePose> poses =
        rest_nodes;

    for (const Track& track : clip.tracks) {
        if (track.node < 0
            || static_cast<std::size_t>(
                   track.node)
                >= poses.size()) {
            continue;
        }

        NodePose& pose =
            poses[track.node];

        pose.matrix_mode = false;

        const auto value =
            sample_track(
                track,
                time);

        switch (track.path) {
        case TrackPath::Translation:
            pose.translation = {
                value[0],
                value[1],
                value[2]
            };
            break;
        case TrackPath::Rotation:
            pose.rotation = {
                value[0],
                value[1],
                value[2],
                value[3]
            };
            break;
        case TrackPath::Scale:
            pose.scale = {
                value[0],
                value[1],
                value[2]
            };
            break;
        }
    }

    std::vector<Mat4> globals(
        poses.size(),
        identity());

    std::vector<std::uint8_t> state(
        poses.size(),
        0u);

    const auto compute_global =
        [&](auto&& self,
            std::size_t index) -> const Mat4& {

        if (state[index] == 2u) {
            return globals[index];
        }

        if (state[index] == 1u) {
            throw std::runtime_error(
                "cycle in character node hierarchy");
        }

        state[index] = 1u;

        const NodePose& pose =
            poses[index];

        const Mat4 local =
            pose.matrix_mode
            ? pose.matrix
            : trs(
                pose.translation,
                pose.rotation,
                pose.scale);

        if (pose.parent >= 0) {
            globals[index] =
                multiply(
                    self(
                        self,
                        static_cast<std::size_t>(
                            pose.parent)),
                    local);
        } else {
            globals[index] =
                local;
        }

        state[index] = 2u;
        return globals[index];
    };

    for (std::size_t i = 0;
         i < poses.size();
         ++i) {
        (void)compute_global(
            compute_global,
            i);
    }

    return globals;
}

int resolve_joint_fragment(
    const std::vector<NodePose>& nodes,
    const std::string& fragment) {

    const std::string needle =
        lower_copy(fragment);

    int root = -1;

    for (std::size_t i = 0;
         i < nodes.size();
         ++i) {

        const std::string name =
            lower_copy(nodes[i].name);

        if (name == needle) {
            return static_cast<int>(i);
        }

        if (name.find(needle)
            != std::string::npos) {

            if (root >= 0) {
                throw std::out_of_range(
                    "joint fragment is ambiguous: "
                    + fragment);
            }

            root =
                static_cast<int>(i);
        }
    }

    if (root < 0) {
        throw std::out_of_range(
            "joint not found: "
            + fragment);
    }

    return root;
}

bool node_descends_from(
    const std::vector<NodePose>& nodes,
    int node,
    int root) {

    std::size_t guard = 0;

    while (node >= 0) {
        if (++guard > nodes.size()) {
            throw std::runtime_error(
                "cycle in character node hierarchy");
        }

        if (node == root) {
            return true;
        }

        node =
            nodes[
                static_cast<std::size_t>(node)]
                .parent;
    }

    return false;
}

} // namespace

GltfCharacter::GltfCharacter()
    : impl_(std::make_unique<Impl>()) {}

GltfCharacter::~GltfCharacter() = default;
GltfCharacter::GltfCharacter(GltfCharacter&&) noexcept = default;
GltfCharacter& GltfCharacter::operator=(GltfCharacter&&) noexcept = default;

void GltfCharacter::load(
    const std::string& character_glb,
    const std::string& animation_glb) {

    auto next = std::make_unique<Impl>();

    CgltfOwner character = load_gltf(character_glb);
    CgltfOwner animation = load_gltf(animation_glb);

    const cgltf_data* char_data = character.data;
    const cgltf_data* anim_data = animation.data;

    next->rest_nodes.resize(char_data->nodes_count);

    for (cgltf_size i = 0;
         i < char_data->nodes_count;
         ++i) {

        const cgltf_node& node = char_data->nodes[i];
        NodePose pose;
        pose.parent = node_index(char_data, node.parent);
        pose.name =
            node.name
            ? std::string(node.name)
            : std::string();

        if (node.has_matrix) {
            pose.matrix_mode = true;
            for (int k = 0; k < 16; ++k) {
                pose.matrix.m[k] =
                    static_cast<double>(node.matrix[k]);
            }
        } else {
            if (node.has_translation) {
                pose.translation = {
                    node.translation[0],
                    node.translation[1],
                    node.translation[2]
                };
            }

            if (node.has_rotation) {
                pose.rotation = {
                    node.rotation[0],
                    node.rotation[1],
                    node.rotation[2],
                    node.rotation[3]
                };
            }

            if (node.has_scale) {
                pose.scale = {
                    node.scale[0],
                    node.scale[1],
                    node.scale[2]
                };
            }
        }

        next->rest_nodes[i] = pose;

        if (!pose.name.empty()) {
            next->node_by_name.emplace(
                lower_copy(pose.name),
                static_cast<int>(i));
        }
    }

    next->skins.resize(char_data->skins_count);

    for (cgltf_size skin_i = 0;
         skin_i < char_data->skins_count;
         ++skin_i) {

        const cgltf_skin& source =
            char_data->skins[skin_i];

        SkinData skin;
        skin.joints.reserve(source.joints_count);
        skin.inverse_bind.reserve(source.joints_count);

        for (cgltf_size joint_i = 0;
             joint_i < source.joints_count;
             ++joint_i) {

            skin.joints.push_back(
                node_index(
                    char_data,
                    source.joints[joint_i]));

            Mat4 inverse = identity();

            if (source.inverse_bind_matrices
                && joint_i
                    < source.inverse_bind_matrices->count) {

                cgltf_float values[16]{};
                if (!cgltf_accessor_read_float(
                        source.inverse_bind_matrices,
                        joint_i,
                        values,
                        16)) {
                    throw std::runtime_error(
                        "failed to read inverse bind matrix");
                }

                for (int k = 0; k < 16; ++k) {
                    inverse.m[k] = values[k];
                }
            }

            skin.inverse_bind.push_back(inverse);
        }

        next->skins[skin_i] = std::move(skin);
    }

    for (cgltf_size node_i = 0;
         node_i < char_data->nodes_count;
         ++node_i) {

        const cgltf_node& node =
            char_data->nodes[node_i];

        if (!node.mesh) continue;

        const int part_skin =
            skin_index(char_data, node.skin);

        for (cgltf_size primitive_i = 0;
             primitive_i < node.mesh->primitives_count;
             ++primitive_i) {

            const cgltf_primitive& primitive =
                node.mesh->primitives[primitive_i];

            if (primitive.type
                != cgltf_primitive_type_triangles) {
                continue;
            }

            const cgltf_accessor* positions = nullptr;
            const cgltf_accessor* joints = nullptr;
            const cgltf_accessor* weights = nullptr;

            for (cgltf_size attr_i = 0;
                 attr_i < primitive.attributes_count;
                 ++attr_i) {

                const cgltf_attribute& attr =
                    primitive.attributes[attr_i];

                if (attr.type
                    == cgltf_attribute_type_position) {
                    positions = attr.data;
                } else if (
                    attr.type
                    == cgltf_attribute_type_joints
                    && attr.index == 0) {
                    joints = attr.data;
                } else if (
                    attr.type
                    == cgltf_attribute_type_weights
                    && attr.index == 0) {
                    weights = attr.data;
                }
            }

            if (!positions) {
                continue;
            }

            MeshPart part;
            part.node = static_cast<int>(node_i);
            part.skin = part_skin;
            part.vertices.resize(positions->count);

            for (cgltf_size vertex_i = 0;
                 vertex_i < positions->count;
                 ++vertex_i) {

                cgltf_float p[3]{};
                if (!cgltf_accessor_read_float(
                        positions,
                        vertex_i,
                        p,
                        3)) {
                    throw std::runtime_error(
                        "failed to read character position");
                }

                VertexData vertex;
                vertex.position = {p[0], p[1], p[2]};

                if (part.skin >= 0
                    && joints
                    && weights) {

                    cgltf_uint j[4]{};
                    cgltf_float w[4]{};

                    if (!cgltf_accessor_read_uint(
                            joints,
                            vertex_i,
                            j,
                            4)
                        || !cgltf_accessor_read_float(
                            weights,
                            vertex_i,
                            w,
                            4)) {
                        throw std::runtime_error(
                            "failed to read character skin weights");
                    }

                    double total = 0.0;
                    for (int k = 0; k < 4; ++k) {
                        vertex.joints[k] =
                            static_cast<std::uint32_t>(j[k]);
                        vertex.weights[k] =
                            static_cast<double>(w[k]);
                        total += vertex.weights[k];
                    }

                    if (total > 1e-12) {
                        for (double& weight : vertex.weights) {
                            weight /= total;
                        }
                        vertex.skinned = true;
                    }
                }

                part.vertices[vertex_i] = vertex;
            }

            if (primitive.indices) {
                part.indices.resize(
                    primitive.indices->count);

                for (cgltf_size index_i = 0;
                     index_i < primitive.indices->count;
                     ++index_i) {

                    part.indices[index_i] =
                        static_cast<std::uint32_t>(
                            cgltf_accessor_read_index(
                                primitive.indices,
                                index_i));
                }
            } else {
                part.indices.resize(
                    part.vertices.size());

                for (std::size_t i = 0;
                     i < part.indices.size();
                     ++i) {
                    part.indices[i] =
                        static_cast<std::uint32_t>(i);
                }
            }

            next->stats.vertices +=
                part.vertices.size();
            next->stats.triangles +=
                part.indices.size() / 3;

            next->parts.push_back(std::move(part));
        }
    }

    next->stats.character_nodes =
        next->rest_nodes.size();

    {
        std::vector<std::uint8_t> seen(
            next->rest_nodes.size(),
            0u);

        for (const auto& skin : next->skins) {
            for (const int joint : skin.joints) {
                if (joint < 0
                    || static_cast<std::size_t>(joint)
                        >= seen.size()
                    || seen[joint]) {
                    continue;
                }

                seen[joint] = 1u;

                CharacterJointInfo info;
                info.name =
                    next->rest_nodes[joint].name;

                const int parent =
                    next->rest_nodes[joint].parent;

                if (parent >= 0
                    && static_cast<std::size_t>(parent)
                        < next->rest_nodes.size()) {
                    info.parent =
                        next->rest_nodes[parent].name;
                }

                cgltf_float world[16]{};
                cgltf_node_transform_world(
                    &char_data->nodes[joint],
                    world);

                info.rest_world_position = {
                    world[12],
                    world[13],
                    world[14]
                };

                next->joint_infos.push_back(
                    std::move(info));
            }
        }

        next->stats.skin_joints =
            next->joint_infos.size();
    }

    next->clips.reserve(anim_data->animations_count);
    next->animation_names.reserve(
        anim_data->animations_count);

    for (cgltf_size animation_i = 0;
         animation_i < anim_data->animations_count;
         ++animation_i) {

        const cgltf_animation& source =
            anim_data->animations[animation_i];

        Clip clip;
        clip.name =
            source.name
            ? std::string(source.name)
            : ("Animation_" + std::to_string(animation_i));

        for (cgltf_size channel_i = 0;
             channel_i < source.channels_count;
             ++channel_i) {

            const cgltf_animation_channel& channel =
                source.channels[channel_i];

            if (!channel.target_node
                || !channel.target_node->name
                || !channel.sampler
                || !channel.sampler->input
                || !channel.sampler->output) {
                continue;
            }

            const auto found =
                next->node_by_name.find(
                    lower_copy(channel.target_node->name));

            if (found == next->node_by_name.end()) {
                continue;
            }

            Track track;
            track.node = found->second;

            std::size_t components = 0;

            switch (channel.target_path) {
            case cgltf_animation_path_type_translation:
                track.path = TrackPath::Translation;
                components = 3;
                break;
            case cgltf_animation_path_type_rotation:
                track.path = TrackPath::Rotation;
                components = 4;
                break;
            case cgltf_animation_path_type_scale:
                track.path = TrackPath::Scale;
                components = 3;
                break;
            default:
                continue;
            }

            switch (channel.sampler->interpolation) {
            case cgltf_interpolation_type_step:
                track.interpolation =
                    TrackInterpolation::Step;
                break;
            case cgltf_interpolation_type_cubic_spline:
                track.interpolation =
                    TrackInterpolation::Cubic;
                break;
            default:
                track.interpolation =
                    TrackInterpolation::Linear;
                break;
            }

            const cgltf_accessor* input =
                channel.sampler->input;
            const cgltf_accessor* output =
                channel.sampler->output;

            track.times.resize(input->count);
            track.values.resize(input->count);

            for (cgltf_size key = 0;
                 key < input->count;
                 ++key) {

                cgltf_float time_value[1]{};
                if (!cgltf_accessor_read_float(
                        input,
                        key,
                        time_value,
                        1)) {
                    throw std::runtime_error(
                        "failed to read animation time");
                }

                track.times[key] = time_value[0];
                clip.duration =
                    std::max(
                        clip.duration,
                        track.times[key]);

                const cgltf_size output_index =
                    track.interpolation
                        == TrackInterpolation::Cubic
                    ? key * 3 + 1
                    : key;

                if (output_index >= output->count) {
                    throw std::runtime_error(
                        "animation output accessor is shorter than expected");
                }

                track.values[key] =
                    read_value(
                        output,
                        output_index,
                        components);
            }

            if (!track.times.empty()) {
                clip.tracks.push_back(
                    std::move(track));
            }
        }

        if (!clip.tracks.empty()) {
            next->animation_names.push_back(
                clip.name);
            next->clips.push_back(
                std::move(clip));
        }
    }

    next->stats.animation_clips =
        next->clips.size();

    if (next->parts.empty()
        || next->stats.vertices == 0
        || next->stats.triangles == 0) {
        throw std::runtime_error(
            "character GLB contains no triangle mesh");
    }

    if (next->clips.empty()) {
        throw std::runtime_error(
            "animation GLB contains no clips that map to the character rig by name");
    }

    impl_ = std::move(next);
}

const CharacterAssetStats& GltfCharacter::stats() const {
    return impl_->stats;
}

const std::vector<std::string>&
GltfCharacter::animation_names() const {
    return impl_->animation_names;
}

const std::vector<CharacterJointInfo>&
GltfCharacter::skin_joints() const {
    return impl_->joint_infos;
}

std::size_t GltfCharacter::find_animation(
    const std::string& name_fragment) const {

    const std::string needle =
        lower_copy(name_fragment);

    for (std::size_t i = 0;
         i < impl_->clips.size();
         ++i) {

        if (lower_copy(impl_->clips[i].name)
                .find(needle)
            != std::string::npos) {
            return i;
        }
    }

    throw std::out_of_range(
        "animation not found: " + name_fragment);
}

double GltfCharacter::animation_duration(
    std::size_t animation) const {

    if (animation >= impl_->clips.size()) {
        throw std::out_of_range(
            "animation index out of range");
    }
    return impl_->clips[animation].duration;
}

std::vector<std::string>
GltfCharacter::animation_target_nodes(
    std::size_t animation) const {

    if (animation >= impl_->clips.size()) {
        throw std::out_of_range(
            "animation index out of range");
    }

    std::vector<std::string> nodes;

    for (const auto& track
         : impl_->clips[animation].tracks) {

        if (track.node < 0
            || static_cast<std::size_t>(
                track.node)
                >= impl_->rest_nodes.size()) {
            continue;
        }

        const std::string& name =
            impl_->rest_nodes[
                static_cast<std::size_t>(
                    track.node)]
                .name;

        if (std::find(
                nodes.begin(),
                nodes.end(),
                name)
            == nodes.end()) {

            nodes.push_back(name);
        }
    }

    return nodes;
}

CharacterMeshFrame GltfCharacter::sample(
    std::size_t animation,
    double time_seconds,
    bool loop,
    const Vec3& world_offset) const {

    if (animation >= impl_->clips.size()) {
        throw std::out_of_range(
            "animation index out of range");
    }

    const Clip& clip = impl_->clips[animation];

    double time = time_seconds;

    if (clip.duration > 1e-12) {
        if (loop) {
            time = std::fmod(
                std::max(0.0, time),
                clip.duration);
        } else {
            time = std::clamp(
                time,
                0.0,
                clip.duration);
        }
    }

    std::vector<NodePose> poses =
        impl_->rest_nodes;

    for (const Track& track : clip.tracks) {
        if (track.node < 0
            || static_cast<std::size_t>(track.node)
                >= poses.size()) {
            continue;
        }

        NodePose& pose = poses[track.node];

        // The Quaternius universal rig uses TRS nodes for animated joints.
        // If a mapped node used a matrix, switching to TRS here is still
        // preferable to silently ignoring an animation channel.
        pose.matrix_mode = false;

        const auto value =
            sample_track(track, time);

        switch (track.path) {
        case TrackPath::Translation:
            pose.translation = {
                value[0],
                value[1],
                value[2]
            };
            break;
        case TrackPath::Rotation:
            pose.rotation = {
                value[0],
                value[1],
                value[2],
                value[3]
            };
            break;
        case TrackPath::Scale:
            pose.scale = {
                value[0],
                value[1],
                value[2]
            };
            break;
        }
    }

    std::vector<Mat4> globals(
        poses.size(),
        identity());
    std::vector<std::uint8_t> state(
        poses.size(),
        0u);

    const auto compute_global =
        [&](auto&& self, std::size_t index) -> const Mat4& {

        if (state[index] == 2u) {
            return globals[index];
        }
        if (state[index] == 1u) {
            throw std::runtime_error(
                "cycle in character node hierarchy");
        }

        state[index] = 1u;

        const NodePose& pose = poses[index];
        const Mat4 local =
            pose.matrix_mode
            ? pose.matrix
            : trs(
                pose.translation,
                pose.rotation,
                pose.scale);

        if (pose.parent >= 0) {
            globals[index] =
                multiply(
                    self(
                        self,
                        static_cast<std::size_t>(
                            pose.parent)),
                    local);
        } else {
            globals[index] = local;
        }

        state[index] = 2u;
        return globals[index];
    };

    for (std::size_t i = 0;
         i < poses.size();
         ++i) {
        (void)compute_global(
            compute_global,
            i);
    }

    CharacterMeshFrame frame;

    frame.positions.reserve(
        impl_->stats.vertices);
    frame.indices.reserve(
        impl_->stats.triangles * 3);

    for (const MeshPart& part : impl_->parts) {
        const std::uint32_t base =
            static_cast<std::uint32_t>(
                frame.positions.size());

        const bool has_skin =
            part.skin >= 0
            && static_cast<std::size_t>(part.skin)
                < impl_->skins.size();

        for (const VertexData& vertex : part.vertices) {
            Vec3 world{};

            if (has_skin && vertex.skinned) {
                const SkinData& skin =
                    impl_->skins[part.skin];

                double total = 0.0;

                for (int influence = 0;
                     influence < 4;
                     ++influence) {

                    const double weight =
                        vertex.weights[influence];

                    if (weight <= 1e-12) {
                        continue;
                    }

                    const std::size_t joint_slot =
                        vertex.joints[influence];

                    if (joint_slot
                        >= skin.joints.size()
                        || joint_slot
                        >= skin.inverse_bind.size()) {
                        continue;
                    }

                    const int joint_node =
                        skin.joints[joint_slot];

                    if (joint_node < 0
                        || static_cast<std::size_t>(
                               joint_node)
                            >= globals.size()) {
                        continue;
                    }

                    const Mat4 skin_matrix =
                        multiply(
                            globals[joint_node],
                            skin.inverse_bind[joint_slot]);

                    world +=
                        transform_point(
                            skin_matrix,
                            vertex.position)
                        * weight;
                    total += weight;
                }

                if (total <= 1e-12) {
                    world =
                        transform_point(
                            globals[part.node],
                            vertex.position);
                } else if (
                    std::abs(total - 1.0) > 1e-8) {
                    world = world / total;
                }
            } else {
                world =
                    transform_point(
                        globals[part.node],
                        vertex.position);
            }

            frame.positions.push_back(
                world + world_offset);
        }

        for (const std::uint32_t index
             : part.indices) {
            frame.indices.push_back(
                base + index);
        }
    }

    return frame;
}

CharacterSplitFrame GltfCharacter::sample_split_branch(
    std::size_t animation,
    double time_seconds,
    const std::string& detached_root_joint_fragment,
    bool loop,
    const Vec3& world_offset) const {

    if (detached_root_joint_fragment.empty()) {
        throw std::invalid_argument(
            "detached root joint fragment must not be empty");
    }

    const std::string needle =
        lower_copy(detached_root_joint_fragment);

    int root_node = -1;

    for (std::size_t i = 0;
         i < impl_->rest_nodes.size();
         ++i) {

        const std::string name =
            lower_copy(impl_->rest_nodes[i].name);

        if (name == needle) {
            root_node = static_cast<int>(i);
            break;
        }
    }

    if (root_node < 0) {
        std::vector<int> matches;

        for (std::size_t i = 0;
             i < impl_->rest_nodes.size();
             ++i) {

            if (lower_copy(
                    impl_->rest_nodes[i].name)
                    .find(needle)
                != std::string::npos) {
                matches.push_back(
                    static_cast<int>(i));
            }
        }

        if (matches.size() != 1) {
            throw std::out_of_range(
                matches.empty()
                ? "detached root joint not found: "
                    + detached_root_joint_fragment
                : "detached root joint fragment is ambiguous: "
                    + detached_root_joint_fragment);
        }

        root_node = matches.front();
    }

    std::vector<std::uint8_t> branch_node(
        impl_->rest_nodes.size(),
        0u);

    for (std::size_t i = 0;
         i < impl_->rest_nodes.size();
         ++i) {

        int current = static_cast<int>(i);
        std::size_t guard = 0;

        while (current >= 0) {
            if (++guard
                > impl_->rest_nodes.size()) {
                throw std::runtime_error(
                    "cycle in character node hierarchy");
            }

            if (current == root_node) {
                branch_node[i] = 1u;
                break;
            }

            current =
                impl_->rest_nodes[current].parent;
        }
    }

    CharacterMeshFrame full =
        sample(
            animation,
            time_seconds,
            loop,
            world_offset);

    std::vector<std::uint8_t> detached_vertex(
        full.positions.size(),
        0u);

    std::size_t vertex_base = 0;

    for (const MeshPart& part : impl_->parts) {
        const bool has_skin =
            part.skin >= 0
            && static_cast<std::size_t>(part.skin)
                < impl_->skins.size();

        for (std::size_t local = 0;
             local < part.vertices.size();
             ++local) {

            const VertexData& vertex =
                part.vertices[local];

            double branch_weight = 0.0;

            if (has_skin && vertex.skinned) {
                const SkinData& skin =
                    impl_->skins[part.skin];

                for (int influence = 0;
                     influence < 4;
                     ++influence) {

                    const double weight =
                        vertex.weights[influence];

                    if (weight <= 1e-12) {
                        continue;
                    }

                    const std::size_t joint_slot =
                        vertex.joints[influence];

                    if (joint_slot
                        >= skin.joints.size()) {
                        continue;
                    }

                    const int node =
                        skin.joints[joint_slot];

                    if (node >= 0
                        && static_cast<std::size_t>(node)
                            < branch_node.size()
                        && branch_node[node]) {
                        branch_weight += weight;
                    }
                }
            }

            if (branch_weight >= 0.5) {
                detached_vertex[
                    vertex_base + local] = 1u;
            }
        }

        vertex_base += part.vertices.size();
    }

    if (vertex_base != full.positions.size()) {
        throw std::logic_error(
            "skin partition vertex count does not match sampled mesh");
    }

    CharacterSplitFrame result;
    result.body.positions = full.positions;
    result.detached.positions = full.positions;

    result.body.indices.reserve(
        full.indices.size());
    result.detached.indices.reserve(
        full.indices.size() / 4);

    for (std::size_t tri = 0;
         tri + 2 < full.indices.size();
         tri += 3) {

        const std::uint32_t i0 =
            full.indices[tri + 0];
        const std::uint32_t i1 =
            full.indices[tri + 1];
        const std::uint32_t i2 =
            full.indices[tri + 2];

        if (i0 >= detached_vertex.size()
            || i1 >= detached_vertex.size()
            || i2 >= detached_vertex.size()) {
            continue;
        }

        const int count =
            static_cast<int>(detached_vertex[i0])
            + static_cast<int>(detached_vertex[i1])
            + static_cast<int>(detached_vertex[i2]);

        if (count == 3) {
            result.detached.indices.insert(
                result.detached.indices.end(),
                {i0, i1, i2});
        } else if (count == 0) {
            result.body.indices.insert(
                result.body.indices.end(),
                {i0, i1, i2});
        } else {
            ++result.boundary_triangles_removed;
        }
    }

    if (result.detached.indices.empty()) {
        throw std::runtime_error(
            "selected joint branch owns no complete mesh triangles");
    }

    return result;
}

std::vector<CharacterPointBinding>
GltfCharacter::bind_points_to_skin(
    std::size_t animation,
    double time_seconds,
    const std::vector<Vec3>& world_points,
    bool loop) const {

    if (animation >= impl_->clips.size()) {
        throw std::out_of_range(
            "animation index out of range");
    }

    const CharacterMeshFrame sampled =
        sample(
            animation,
            time_seconds,
            loop);

    const auto globals =
        sampled_globals(
            impl_->rest_nodes,
            impl_->clips[animation],
            time_seconds,
            loop);

    std::vector<CharacterPointBinding>
        bindings;

    bindings.reserve(
        world_points.size());

    for (const Vec3& point
         : world_points) {

        double best_distance =
            std::numeric_limits<double>::infinity();

        const MeshPart* best_part =
            nullptr;

        const VertexData* best_vertex =
            nullptr;

        std::size_t global_vertex = 0;

        for (const MeshPart& part
             : impl_->parts) {

            for (const VertexData& vertex
                 : part.vertices) {

                if (global_vertex
                    >= sampled.positions.size()) {
                    throw std::logic_error(
                        "sampled vertex count mismatch");
                }

                if (vertex.skinned) {
                    const double distance =
                        length_squared(
                            sampled.positions[
                                global_vertex]
                            - point);

                    if (distance
                        < best_distance) {
                        best_distance =
                            distance;
                        best_part = &part;
                        best_vertex = &vertex;
                    }
                }

                ++global_vertex;
            }
        }

        if (!best_part
            || !best_vertex
            || best_part->skin < 0
            || static_cast<std::size_t>(
                   best_part->skin)
                >= impl_->skins.size()) {
            throw std::runtime_error(
                "could not bind character point to skinned vertex");
        }

        const SkinData& skin =
            impl_->skins[
                static_cast<std::size_t>(
                    best_part->skin)];

        CharacterPointBinding binding;

        double total = 0.0;
        double strongest = -1.0;

        for (int influence = 0;
             influence < 4;
             ++influence) {

            const double weight =
                best_vertex
                    ->weights[influence];

            if (weight <= 1e-12) {
                continue;
            }

            const std::size_t slot =
                best_vertex
                    ->joints[influence];

            if (slot >= skin.joints.size()) {
                continue;
            }

            const int joint_node =
                skin.joints[slot];

            if (joint_node < 0
                || static_cast<std::size_t>(
                       joint_node)
                    >= globals.size()) {
                continue;
            }

            CharacterPointInfluence out;
            out.joint_node = joint_node;
            out.joint_name =
                impl_->rest_nodes[
                    static_cast<std::size_t>(
                        joint_node)]
                    .name;

            out.weight = weight;
            out.joint_local_point =
                transform_point(
                    inverse_affine(
                        globals[
                            static_cast<std::size_t>(
                                joint_node)]),
                    point);

            binding.influences[
                binding.influence_count++] =
                    out;

            total += weight;

            if (weight > strongest) {
                strongest = weight;
                binding.dominant_joint =
                    out.joint_name;
            }
        }

        if (binding.influence_count == 0
            || total <= 1e-12) {
            throw std::runtime_error(
                "character point binding has no valid influences");
        }

        if (std::abs(total - 1.0)
            > 1e-8) {
            for (std::size_t i = 0;
                 i < binding.influence_count;
                 ++i) {
                binding.influences[i]
                    .weight /= total;
            }
        }

        bindings.push_back(
            std::move(binding));
    }

    return bindings;
}

std::vector<Vec3>
GltfCharacter::sample_bound_points(
    const std::vector<CharacterPointBinding>& bindings,
    std::size_t animation,
    double time_seconds,
    bool loop,
    const Vec3& world_offset) const {

    if (animation >= impl_->clips.size()) {
        throw std::out_of_range(
            "animation index out of range");
    }

    const auto globals =
        sampled_globals(
            impl_->rest_nodes,
            impl_->clips[animation],
            time_seconds,
            loop);

    std::vector<Vec3> points;
    points.reserve(
        bindings.size());

    for (const auto& binding
         : bindings) {

        Vec3 point{};
        double total = 0.0;

        for (std::size_t i = 0;
             i < binding.influence_count;
             ++i) {

            const auto& influence =
                binding.influences[i];

            if (influence.joint_node < 0
                || static_cast<std::size_t>(
                       influence.joint_node)
                    >= globals.size()
                || influence.weight <= 1e-12) {
                continue;
            }

            point +=
                transform_point(
                    globals[
                        static_cast<std::size_t>(
                            influence.joint_node)],
                    influence.joint_local_point)
                * influence.weight;

            total += influence.weight;
        }

        if (total <= 1e-12) {
            throw std::runtime_error(
                "bound character point has no valid sampled influence");
        }

        if (std::abs(total - 1.0)
            > 1e-8) {
            point = point / total;
        }

        points.push_back(
            point + world_offset);
    }

    return points;
}

double GltfCharacter::binding_branch_weight(
    const CharacterPointBinding& binding,
    const std::string& root_joint_fragment) const {

    const int root =
        resolve_joint_fragment(
            impl_->rest_nodes,
            root_joint_fragment);

    double total = 0.0;

    for (std::size_t i = 0;
         i < binding.influence_count;
         ++i) {

        const auto& influence =
            binding.influences[i];

        if (influence.joint_node < 0
            || influence.weight <= 1e-12) {
            continue;
        }

        if (node_descends_from(
                impl_->rest_nodes,
                influence.joint_node,
                root)) {
            total += influence.weight;
        }
    }

    return total;
}

} // namespace sarx
