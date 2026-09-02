#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <memory>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;

namespace {
    Handle MakeHandle(uint32_t index) {
        return Handle{index, 0};
    }

    // Builds a one-shape-per-body world and runs it through NarrowPhase.
    std::vector<Contact> GenerateContact(std::unique_ptr<IShape> shapeA, const Vector3& posA,
                                          std::unique_ptr<IShape> shapeB, const Vector3& posB) {
        BodyManager bodyManager;
        ShapeManager shapeManager;
        Handle a = MakeHandle(0);
        Handle b = MakeHandle(1);

        bodyManager.AddBody(a, BodyData{posA, Vector3::Zero(), 1.0, 1.0});
        bodyManager.AddBody(b, BodyData{posB, Vector3::Zero(), 1.0, 1.0});
        shapeManager.AddShape(a, std::move(shapeA));
        shapeManager.AddShape(b, std::move(shapeB));

        NarrowPhase narrowPhase;
        std::vector<Contact> contacts;
        narrowPhase.GenerateContacts({CollisionCandidatePair{a, b}}, bodyManager, shapeManager, contacts);
        return contacts;
    }
}

// ==========================================================================
// Point / Point
// ==========================================================================

TEST_CASE("NarrowPhase - Point vs Point: separated points produce no contact", "[narrowphase][matrix]")
{
    auto contacts =
        GenerateContact(std::make_unique<ShapePoint>(), Vector3(0.0, 0.0, 0.0), std::make_unique<ShapePoint>(),
                        Vector3(5.0, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

TEST_CASE("NarrowPhase - Point vs Point: coincident points produce no contact (zero-volume shapes never overlap in GJK's strict sense)", "[narrowphase][matrix]")
{
    // Two zero-extent points at the same location: GJK's simplex never
    // encloses the origin with strictly positive volume, so this is
    // treated the same as an exact-touching boundary case elsewhere in
    // the suite (no confirmed collision).
    auto contacts =
        GenerateContact(std::make_unique<ShapePoint>(), Vector3(0.0, 0.0, 0.0), std::make_unique<ShapePoint>(),
                        Vector3(0.0, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

// ==========================================================================
// Point / Sphere
// ==========================================================================

TEST_CASE("NarrowPhase - Point vs Sphere: point outside sphere produces no contact", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapePoint>(), Vector3(5.0, 0.0, 0.0),
                                    std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

TEST_CASE("NarrowPhase - Point vs Sphere: point just outside the surface produces no contact", "[narrowphase][matrix][boundary]")
{
    auto contacts = GenerateContact(std::make_unique<ShapePoint>(), Vector3(1.0001, 0.0, 0.0),
                                    std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

TEST_CASE("NarrowPhase - Point vs Sphere: point just inside the surface produces a contact", "[narrowphase][matrix][boundary]")
{
    auto contacts = GenerateContact(std::make_unique<ShapePoint>(), Vector3(0.9999, 0.0, 0.0),
                                    std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
}

TEST_CASE("NarrowPhase - Point vs Sphere: point inside the sphere produces a contact", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapePoint>(), Vector3(0.3, 0.0, 0.0),
                                    std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
}

TEST_CASE("NarrowPhase - Point vs Sphere: point deep inside (near center) produces a contact", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapePoint>(), Vector3(0.01, 0.0, 0.0),
                                    std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
}

// ==========================================================================
// Point / Cube
// ==========================================================================

TEST_CASE("NarrowPhase - Point vs Cube: point outside cube produces no contact", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapePoint>(), Vector3(5.0, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(0.0, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

TEST_CASE("NarrowPhase - Point vs Cube: point just outside a face produces no contact", "[narrowphase][matrix][boundary]")
{
    auto contacts = GenerateContact(std::make_unique<ShapePoint>(), Vector3(0.5001, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(0.0, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

TEST_CASE("NarrowPhase - Point vs Cube: point just inside a face produces a contact", "[narrowphase][matrix][boundary]")
{
    auto contacts = GenerateContact(std::make_unique<ShapePoint>(), Vector3(0.4999, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(0.0, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
}

TEST_CASE("NarrowPhase - Point vs Cube: point inside the cube produces a contact", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapePoint>(), Vector3(0.2, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(0.0, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
}

TEST_CASE("NarrowPhase - Point vs Cube: point near the cube's center produces a contact", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapePoint>(), Vector3(0.01, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(0.0, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
}

// ==========================================================================
// Sphere / Sphere
// ==========================================================================

TEST_CASE("NarrowPhase - Sphere vs Sphere: separated spheres produce no contact", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeSphere>(1.0), Vector3(5.0, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

TEST_CASE("NarrowPhase - Sphere vs Sphere: exactly touching spheres produce no contact", "[narrowphase][matrix][boundary]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeSphere>(1.0), Vector3(2.0, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

TEST_CASE("NarrowPhase - Sphere vs Sphere: shallow overlap produces a contact with small penetration", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeSphere>(1.0), Vector3(1.95, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].penetration > 0.0);
    REQUIRE(contacts[0].penetration < 0.2);
}

TEST_CASE("NarrowPhase - Sphere vs Sphere: deep overlap produces a contact with large penetration", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeSphere>(1.0), Vector3(0.5, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
    REQUIRE_THAT(contacts[0].penetration, WithinAbs(1.5, 1e-2));
}

// ==========================================================================
// Sphere / Cube
// ==========================================================================

TEST_CASE("NarrowPhase - Sphere vs Cube: separated shapes produce no contact", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(5.0, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

TEST_CASE("NarrowPhase - Sphere vs Cube: shapes just outside touching distance produce no contact", "[narrowphase][matrix][boundary]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(1.5001, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

TEST_CASE("NarrowPhase - Sphere vs Cube: shapes just inside touching distance produce a contact", "[narrowphase][matrix][boundary]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(1.4999, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
}

TEST_CASE("NarrowPhase - Sphere vs Cube: shallow overlap produces a contact", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(1.4, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].penetration > 0.0);
}

TEST_CASE("NarrowPhase - Sphere vs Cube: deep overlap produces a contact with large penetration", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeSphere>(1.0), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(0.5, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].penetration > 0.5);
}

// ==========================================================================
// Cube / Cube
// ==========================================================================

TEST_CASE("NarrowPhase - Cube vs Cube: separated cubes produce no contact", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeCube>(0.5), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(5.0, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

TEST_CASE("NarrowPhase - Cube vs Cube: exactly face-touching cubes produce no contact", "[narrowphase][matrix][boundary]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeCube>(0.5), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(1.0, 0.0, 0.0));
    REQUIRE(contacts.empty());
}

TEST_CASE("NarrowPhase - Cube vs Cube: shallow overlap produces a contact with small penetration", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeCube>(0.5), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(0.95, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
    REQUIRE_THAT(contacts[0].penetration, WithinAbs(0.05, 1e-6));
}

TEST_CASE("NarrowPhase - Cube vs Cube: deep overlap produces a contact with large penetration", "[narrowphase][matrix]")
{
    auto contacts = GenerateContact(std::make_unique<ShapeCube>(0.5), Vector3(0.0, 0.0, 0.0),
                                    std::make_unique<ShapeCube>(0.5), Vector3(0.2, 0.0, 0.0));
    REQUIRE(contacts.size() == 1);
    REQUIRE_THAT(contacts[0].penetration, WithinAbs(0.8, 1e-6));
}

// ==========================================================================
// Pair ordering
// ==========================================================================

TEST_CASE("NarrowPhase - (A, B) and (B, A) candidate order produce the same collision state", "[narrowphase][symmetry]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);

    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(1.5, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    NarrowPhase narrowPhase;

    std::vector<Contact> contactsAB;
    narrowPhase.GenerateContacts({CollisionCandidatePair{a, b}}, bodyManager, shapeManager, contactsAB);

    std::vector<Contact> contactsBA;
    narrowPhase.GenerateContacts({CollisionCandidatePair{b, a}}, bodyManager, shapeManager, contactsBA);

    REQUIRE(contactsAB.size() == contactsBA.size());
    REQUIRE(contactsAB.size() == 1);
}

TEST_CASE("NarrowPhase - Swapping candidate pair order flips the contact normal but keeps its magnitude", "[narrowphase][symmetry]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);

    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(1.5, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    NarrowPhase narrowPhase;

    std::vector<Contact> contactsAB;
    narrowPhase.GenerateContacts({CollisionCandidatePair{a, b}}, bodyManager, shapeManager, contactsAB);

    std::vector<Contact> contactsBA;
    narrowPhase.GenerateContacts({CollisionCandidatePair{b, a}}, bodyManager, shapeManager, contactsBA);

    REQUIRE(contactsAB[0].a.index == a.index);
    REQUIRE(contactsBA[0].a.index == b.index);

    // Normal always points from "a" (whichever handle is first in the
    // pair) toward "b" - so the two runs should report opposite normals
    // of the same magnitude. EPA is an iterative polytope-expansion
    // algorithm, so two independently-run resolves (different starting
    // simplex order) leave a small residual difference larger than exact
    // equality - tolerance reflects that, not a precision guarantee.
    REQUIRE_THAT(contactsAB[0].normal.x, WithinAbs(-contactsBA[0].normal.x, 1e-3));
    REQUIRE_THAT(contactsAB[0].normal.y, WithinAbs(-contactsBA[0].normal.y, 1e-3));
    REQUIRE_THAT(contactsAB[0].normal.z, WithinAbs(-contactsBA[0].normal.z, 1e-3));
    REQUIRE_THAT(contactsAB[0].penetration, WithinAbs(contactsBA[0].penetration, 1e-3));
}
