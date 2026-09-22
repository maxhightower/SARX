#pragma once

// Layered anatomical voxel body (V0.5).
//
// Physical model: every voxel owns eight corner particles held in shape by a
// Gram-Schmidt voxel (VGS) constraint, and neighbouring voxels are joined by
// breakable zero-rest-length face bonds between coincident corners. This is the
// constraint layout of McGraw (MIG 2024) with the vertex-coincident face
// constraints described by Lin (Purdue M.S. thesis 2025, Algorithm 2 and
// section 3.6). Because a voxel owns its corners, all VGS constraints are
// mutually independent and all face bonds along one axis are mutually
// independent, so one solver iteration is exactly four conflict-free parallel
// passes (voxels, +X bonds, +Y bonds, +Z bonds).
//
// SARX additions on top of that layout (not claimed by the cited work):
//  - tissue layers (skin, fat, muscle, bone, marrow, organs) with per-tissue
//    stiffness, bond hit points, tear strain and ballistic resistance,
//  - persistent partial bond damage, so a blade that runs out of energy lodges
//    and a later stroke continues the same wound,
//  - energy-ordered blade sweeps and energy-ordered ballistic penetration that
//    report an exit state for the surrounding world,
//  - skinned rig targets applied as compliant per-tissue "tone", with rig
//    authority removed from any component that loses the root.

#include "sarx/math.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace sarx {

enum class Tissue : std::uint8_t {
    Skin,
    Fat,
    Muscle,
    Tendon,
    Bone,
    Marrow,
    Brain,
    Heart,
    Lung,
    Liver,
    Gut,
    Count
};

inline constexpr std::size_t kTissueCount =
    static_cast<std::size_t>(Tissue::Count);

[[nodiscard]] const char* tissue_name(Tissue tissue);

struct Rgb8 {
    std::uint8_t r{};
    std::uint8_t g{};
    std::uint8_t b{};
};

struct TissueProperties {
    // Mass density (kg/m^3).
    float density{1050.0f};

    // Fraction of the way each voxel's corners move toward the converged VGS
    // goal shape per projection, in (0, 1]. Lin 2025 (Alg. 2) instead relaxes
    // the Gram-Schmidt, edge-length and volume steps separately (alpha, beta,
    // delta); in SARX testing that split is not contractive for resting
    // contact stacks at low iteration counts, whereas a single blend toward a
    // fully projected goal is (standard PBD stiffness).
    float shape_stiffness{0.6f};

    // Position-projection stiffness of face bonds in [0, 1].
    float bond_stiffness{1.0f};

    // Bond hit points against a sharp edge (Joules to part one bond).
    float cut_hp{2.0f};

    // Corner separation, in voxel edge lengths, above which a bond is
    // overloaded. Overload drains bond HP over a few substeps (so impact
    // transients do not shatter tissue); 3x this separation parts it at once.
    float tear_strain{0.6f};

    // Energy a projectile spends to destroy one voxel of this tissue,
    // normalised to a 2 cm voxel and scaled by voxel area.
    float ballistic_cost{15.0f};

    // Fraction of the rig target applied per substep. Bone is effectively
    // rigidly driven; soft tissue gets secondary motion.
    float rig_tone{0.05f};

    Rgb8 color{200, 120, 110};
};

using TissueTable = std::array<TissueProperties, kTissueCount>;

[[nodiscard]] TissueTable default_tissue_table();

// Affine rest->current transform for one rig joint.
struct JointTransform {
    // Column-major 3x3 rotation/scale.
    std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};
    Vec3 t{};

    [[nodiscard]] Vec3 apply(const Vec3& p) const {
        return {
            m[0] * p.x + m[3] * p.y + m[6] * p.z + t.x,
            m[1] * p.x + m[4] * p.y + m[7] * p.z + t.y,
            m[2] * p.x + m[5] * p.y + m[8] * p.z + t.z
        };
    }
};

inline constexpr std::uint16_t kNoJoint = 0xFFFFu;

// Voxelised anatomy before simulation state exists. Voxels are addressed on a
// regular grid; only occupied cells are listed.
struct AnatomyVoxelDesc {
    std::array<std::int32_t, 3> cell{};
    Tissue tissue{Tissue::Muscle};
    std::array<std::uint16_t, 4> joints{kNoJoint, kNoJoint, kNoJoint, kNoJoint};
    std::array<float, 4> weights{0, 0, 0, 0};
    // Multiplier on the tissue tear strain (e.g. slack skin over joints).
    float tear_scale{1.0f};
};

struct AnatomyDesc {
    Vec3 origin{};
    double voxel_size{0.02};
    std::vector<AnatomyVoxelDesc> voxels;
    std::vector<std::string> joint_names;
    // Joints whose bone voxels confer rig authority on their component.
    std::vector<std::uint16_t> root_joints;
};

struct AnatomyStepConfig {
    int substeps{4};
    // Upper bound for adaptive substepping: the step count rises so that no
    // dynamic voxel moves more than half a voxel per substep (CFL-style).
    int max_substeps{12};
    int iterations{2};
    Vec3 gravity{0.0, -9.81, 0.0};
    double ground_height{0.0};
    double ground_friction{0.6};
    // Per-substep velocity retention (1 = no damping).
    double velocity_retention{0.999};
    // Fraction of each voxel's internal (corner-relative) velocity removed per
    // substep (Mueller et al. 2007, section 3.5, applied per voxel). Damps
    // high-frequency VGS/contact modes without damping bulk motion.
    double deformation_damping{0.3};
    bool enable_tearing{true};
    bool enable_rig{true};
    // Worker threads for the conflict-free solver passes (0 = hardware
    // concurrency, 1 = serial). Results are bit-identical for any value.
    unsigned threads{0};
    // A free island sleeps (zero cost until damage or a grab touches it) once
    // 97% of its voxels have stayed below this speed (m/s) for `sleep_frames`
    // consecutive frames. 0 disables sleeping.
    double sleep_speed{0.08};
    int sleep_frames{45};
    // Voxel-voxel contact between different connected components (severed
    // parts landing on the body, limbs piling up).
    bool enable_component_contacts{true};
    // Rigid shape matching of each intact bone fragment (0 disables).
    double bone_rigidity{1.0};
    // Stacking mass scaling (Macklin et al. 2014, section 5.2): while solving,
    // inverse mass is multiplied by exp(k * height above ground) so load paths
    // to the ground converge in few iterations. 0 disables.
    double stack_mass_scaling{3.0};
};

// Simulation residency. Hybrid keeps intact rig-authoritative voxels kinematic
// (driven by linear-blend skinning, zero solver cost) and simulates only voxels
// activated by damage, grabs, or loss of rig authority. Dynamic simulates all.
enum class AnatomySimMode : std::uint8_t {
    Hybrid,
    Dynamic
};

struct BladePose {
    Vec3 hilt{};
    Vec3 tip{};
};

// A blade moving through a sequence of poses within one frame. Consecutive
// poses form bilinear swept patches. Bonds are processed in the order the edge
// reaches them, and each consumes energy; when energy runs out the blade lodges.
struct BladeStroke {
    std::vector<BladePose> poses;
    double energy{60.0};
    // 1 = razor sharp, lower = heavier/duller edge (bond cost / sharpness).
    double sharpness{1.0};
    // Separation applied across a parted bond (fraction of voxel size).
    double wedge{0.15};
    // Fraction of spent energy transferred as momentum along the swing.
    double momentum_transfer{0.25};
    double duration{1.0 / 60.0};
};

struct BladeResult {
    std::size_t bonds_crossed{};
    std::size_t bonds_parted{};
    std::size_t bonds_damaged{};
    double energy_spent{};
    bool lodged{false};
    // Sweep progress in [0, poses-1] where the blade stopped (or ended).
    double stop_parameter{};
    Vec3 stop_point{};
    Tissue stopped_in{Tissue::Skin};
    std::array<std::size_t, kTissueCount> parted_by_tissue{};
};

struct BulletShot {
    Vec3 origin{};
    Vec3 direction{0.0, 0.0, -1.0};
    double speed{370.0};
    double mass{0.008};
    double radius{0.0045};
    double max_distance{50.0};
};

struct AnatomyDebris {
    Vec3 position{};
    Vec3 velocity{};
    Tissue tissue{Tissue::Muscle};
};

struct BulletResult {
    bool entered{false};
    bool exited{false};
    bool lodged{false};
    Vec3 entry{};
    Vec3 exit_point{};
    Vec3 exit_velocity{};
    double energy_in{};
    double energy_out{};
    std::size_t voxels_destroyed{};
    std::size_t bonds_broken{};
    std::array<std::size_t, kTissueCount> destroyed_by_tissue{};
    std::vector<AnatomyDebris> debris;
};

struct TearEvent {
    std::size_t bond{};
    Vec3 position{};
    Tissue tissue{};
};

struct AnatomyComponent {
    std::vector<std::uint32_t> voxels;
    bool rig_authoritative{false};
    double mass{};
    Vec3 linear_momentum{};
};

struct AnatomyStats {
    std::size_t voxels{};
    std::size_t live_voxels{};
    std::size_t particles{};
    std::size_t bonds{};
    std::size_t live_bonds{};
    std::array<std::size_t, kTissueCount> voxels_by_tissue{};
};

// Grab handle: pulls every live voxel inside a sphere toward a moving target.
struct AnatomyGrab {
    std::vector<std::uint32_t> voxels;
    std::vector<Vec3> offsets;
    Vec3 target{};
    double stiffness{0.5};
    bool active{true};
};

class AnatomyBody {
public:
    AnatomyBody() = default;
    AnatomyBody(const AnatomyDesc& desc, const TissueTable& tissues = default_tissue_table());

    // ---- rig ----------------------------------------------------------
    // Set rest->current transforms for every joint named in the description.
    void set_pose(const std::vector<JointTransform>& joints);
    [[nodiscard]] std::size_t joint_count() const { return joint_names_.size(); }
    [[nodiscard]] const std::vector<std::string>& joint_names() const { return joint_names_; }
    // Snap all particles to the current rig targets (teleport; zero velocity).
    void snap_to_pose();
    // Remove all rig authority (e.g. on death).
    void release_rig();

    // ---- residency ------------------------------------------------------
    void set_sim_mode(AnatomySimMode mode);
    [[nodiscard]] AnatomySimMode sim_mode() const { return mode_; }
    // Promote every live voxel within `radius` of `center` to dynamic.
    std::size_t activate_sphere(const Vec3& center, double radius);
    // Radius promoted around every damage event (metres).
    void set_activation_halo(double radius) { activation_halo_ = radius; }
    [[nodiscard]] bool voxel_dynamic(std::uint32_t v) const { return kinematic_[v] == 0; }
    [[nodiscard]] bool voxel_asleep(std::uint32_t v) const { return kinematic_[v] == 2; }
    [[nodiscard]] std::size_t dynamic_voxel_count() const;

    // ---- simulation ---------------------------------------------------
    void step(double dt, const AnatomyStepConfig& config = {});

    std::size_t add_grab(const Vec3& center, double radius, double stiffness = 0.5);
    void set_grab_target(std::size_t grab, const Vec3& target);
    void release_grab(std::size_t grab);

    // ---- damage -------------------------------------------------------
    [[nodiscard]] BladeResult apply_blade(const BladeStroke& stroke);
    [[nodiscard]] BulletResult apply_bullet(const BulletShot& shot);
    // Sweep strain and tear overstretched bonds (also done inside step()).
    std::size_t tear_overstretched();

    // ---- topology -----------------------------------------------------
    [[nodiscard]] const std::vector<AnatomyComponent>& components();
    [[nodiscard]] std::size_t component_of(std::uint32_t voxel);

    // ---- queries ------------------------------------------------------
    [[nodiscard]] AnatomyStats stats() const;
    [[nodiscard]] std::size_t voxel_count() const { return tissue_.size(); }
    [[nodiscard]] bool voxel_alive(std::uint32_t v) const { return alive_[v] != 0; }
    [[nodiscard]] Tissue voxel_tissue(std::uint32_t v) const { return tissue_[v]; }
    [[nodiscard]] std::array<std::int32_t, 3> voxel_cell(std::uint32_t v) const;
    [[nodiscard]] Vec3 voxel_center(std::uint32_t v) const;
    [[nodiscard]] Vec3 voxel_rest_center(std::uint32_t v) const;
    [[nodiscard]] Vec3 voxel_velocity(std::uint32_t v) const;
    [[nodiscard]] Vec3 corner(std::uint32_t v, int c) const;
    [[nodiscard]] double voxel_size() const { return h_; }
    [[nodiscard]] Vec3 total_linear_momentum() const;
    [[nodiscard]] double total_mass() const;

    // Neighbour voxel across face `axis` (0..2) in direction `dir` (0 = -, 1 = +),
    // or UINT32_MAX when absent.
    [[nodiscard]] std::uint32_t neighbor(std::uint32_t v, int axis, int dir) const;
    // Bond across the same face, or SIZE_MAX when none exists.
    [[nodiscard]] std::size_t bond_across(std::uint32_t v, int axis, int dir) const;
    [[nodiscard]] bool bond_active(std::size_t bond) const { return bond_active_[bond] != 0; }
    [[nodiscard]] float bond_hp(std::size_t bond) const { return bond_hp_[bond]; }
    [[nodiscard]] float bond_max_hp(std::size_t bond) const { return bond_max_hp_[bond]; }
    [[nodiscard]] std::size_t bond_count() const { return bond_a_.size(); }
    [[nodiscard]] std::uint32_t bond_voxel_a(std::size_t b) const { return bond_a_[b]; }
    [[nodiscard]] std::uint32_t bond_voxel_b(std::size_t b) const { return bond_b_[b]; }
    [[nodiscard]] int bond_axis(std::size_t b) const { return bond_axis_[b]; }
    // True when the face has been parted at least once (wound surface).
    [[nodiscard]] bool face_wounded(std::uint32_t v, int axis, int dir) const;

    [[nodiscard]] const std::vector<TearEvent>& recent_tears() const { return recent_tears_; }
    [[nodiscard]] int last_substeps() const { return last_substeps_; }
    [[nodiscard]] const TissueTable& tissues() const { return tissues_; }

    // Order-independent digest of all particle positions and topology flags.
    [[nodiscard]] std::uint64_t state_hash() const;

private:
    void build_bonds();
    void compute_rig_targets();
    void solve_vgs(std::size_t begin, std::size_t end);
    void solve_bond_axis(int axis, std::size_t begin, std::size_t end);
    void compact_active();
    std::size_t tear_active_bonds();
    void apply_rig(double fraction);
    void apply_grabs();
    void collide_ground(const AnatomyStepConfig& config);
    void break_bond(std::size_t bond);
    void kill_voxel(std::uint32_t v);
    void mark_topology_dirty() { topology_dirty_ = true; }
    void rebuild_components();
    void add_voxel_velocity(std::uint32_t v, const Vec3& dv);
    void drive_kinematic(double dt);
    void update_sleep(const AnatomyStepConfig& config);
    void solve_component_contacts();
    void rebuild_bone_clusters();
    void solve_bone_clusters(float stiffness);
    [[nodiscard]] float effective_inv_mass(std::uint32_t v) const {
        return kinematic_[v] ? 0.0f : voxel_inv_mass_[v];
    }
    void offset_voxel(std::uint32_t v, const Vec3& dx);

    TissueTable tissues_{};
    Vec3 origin_{};
    double h_{0.02};
    float r_{0.01f};

    // Grid lookup.
    std::array<std::int32_t, 3> grid_min_{};
    std::array<std::int32_t, 3> grid_dim_{};
    std::vector<std::uint32_t> grid_;

    // Per voxel.
    std::vector<std::array<std::int32_t, 3>> cell_;
    std::vector<Tissue> tissue_;
    std::vector<std::uint8_t> alive_;
    std::vector<std::uint8_t> rig_enabled_;
    std::vector<std::array<std::uint16_t, 4>> joints_;
    std::vector<std::array<float, 4>> weights_;
    std::vector<float> tear_scale_;
    std::vector<float> rest_cx_, rest_cy_, rest_cz_;
    std::vector<float> target_x_, target_y_, target_z_;
    // Blended (LBS) 3x3 per voxel for corner targets, column-major.
    std::vector<std::array<float, 9>> target_m_;
    // 0 = simulated, 1 = rig-driven kinematic, 2 = asleep (frozen free island).
    std::vector<std::uint8_t> kinematic_;
    // Consecutive calm frames per free island, keyed by its lowest voxel id.
    std::unordered_map<std::uint32_t, int> island_calm_;
    AnatomySimMode mode_{AnatomySimMode::Hybrid};
    double activation_halo_{0.08};
    std::vector<float> voxel_inv_mass_;
    // Six face slots per voxel: bond index or UINT32_MAX.
    std::vector<std::array<std::uint32_t, 6>> face_bond_;
    std::vector<std::uint8_t> face_wounds_;

    // Per particle (8 per voxel, corner bit0 = +x, bit1 = +y, bit2 = +z).
    std::vector<float> px_, py_, pz_;
    std::vector<float> ox_, oy_, oz_;
    std::vector<float> vx_, vy_, vz_;

    // Bonds, stored grouped by axis so each axis range is one parallel pass.
    std::vector<std::uint32_t> bond_a_, bond_b_;
    std::vector<std::uint8_t> bond_axis_;
    std::vector<std::uint8_t> bond_active_;
    std::vector<float> bond_hp_, bond_max_hp_, bond_stiffness_, bond_tear_;
    std::array<std::size_t, 4> axis_begin_{};

    // Rig.
    std::vector<std::string> joint_names_;
    std::vector<std::uint16_t> root_joints_;
    std::vector<JointTransform> pose_;
    bool rig_released_{false};

    std::vector<AnatomyGrab> grabs_;

    // Stream-compacted work lists rebuilt each step (Lin 2025, section 3.8).
    std::vector<std::uint32_t> dyn_voxels_;
    std::array<std::vector<std::uint32_t>, 3> dyn_bonds_;

    bool topology_dirty_{true};
    std::vector<AnatomyComponent> components_;
    std::vector<std::uint32_t> component_of_;

    // Bone fragments: bone voxels of one joint connected by intact bone bonds.
    struct BoneCluster {
        std::vector<std::uint32_t> voxels;
        std::vector<Vec3> rest_offsets;
        std::array<double, 4> q{0, 0, 0, 1};  // warm-started rotation (x, y, z, w)
        Vec3 rest_com{};
        Vec3 com{};
        std::array<double, 9> R{1, 0, 0, 0, 1, 0, 0, 0, 1};
        double inv_mass{0.0};
        bool kinematic{false};
        bool grounded{false};
    };
    // Ball joint between two bone fragments that still share an intact
    // articulation (bone-bone bond across different rig joints).
    struct BoneJoint {
        std::uint32_t a{};
        std::uint32_t b{};
        Vec3 rest_anchor{};
    };
    std::vector<BoneCluster> bone_clusters_;
    std::vector<BoneJoint> bone_joints_;
    void solve_bone_joints();
    float ground_height_{0.0f};
    float stack_k_{0.0f};
    std::vector<float> solve_w_;
    void refresh_solve_weights();
    bool bone_clusters_dirty_{true};

    std::vector<TearEvent> recent_tears_;
    int last_substeps_{0};
};

// ---- builders ---------------------------------------------------------------

struct HumanoidAnatomySpec {
    double voxel_size{0.02};
    // Skin and subcutaneous fat thickness in metres.
    double skin_thickness{0.012};
    double fat_thickness{0.015};
    bool ribs{true};
    bool organs{true};
};

// Procedural adult humanoid (~1.8 m) with skeleton, rib cage, skull, spine,
// brain, heart, lungs, liver and gut, wrapped in muscle, fat and skin.
// Joint 0 is the pelvis root.
[[nodiscard]] AnatomyDesc build_humanoid_anatomy(const HumanoidAnatomySpec& spec = {});

// Joint rest positions for the procedural humanoid (index-aligned with
// AnatomyDesc::joint_names), for animation helpers.
[[nodiscard]] std::vector<Vec3> humanoid_anatomy_joint_rest_positions();
[[nodiscard]] std::vector<int> humanoid_anatomy_joint_parents();

// Simple procedural animation over the humanoid skeleton: `walk_phase` in
// radians drives arm/leg swing; `root_offset` translates the whole rig.
[[nodiscard]] std::vector<JointTransform> humanoid_anatomy_pose(
    double walk_phase,
    double swing_amount,
    const Vec3& root_offset = {});

// Rotation helper (axis must be normalised).
[[nodiscard]] JointTransform rotation_about(const Vec3& pivot, const Vec3& axis, double angle);
[[nodiscard]] JointTransform compose(const JointTransform& outer, const JointTransform& inner);

} // namespace sarx
