#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>

#include <memory>
#include <vector>


namespace {

    TEST_CASE("ShapeManager starts empty", "[ShapeManager]") {
        osseus::ShapeManager shapeManager;

        REQUIRE(shapeManager.Data().empty());
        REQUIRE(shapeManager.Handles().empty());
    }


    TEST_CASE("ShapeManager registers a handle",
              "[ShapeManager][Handle][Lifecycle]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle handle{0, 0};

        shapeManager.Register(handle);

        REQUIRE(shapeManager.GetShape(handle) != nullptr);
    }


    TEST_CASE("ShapeManager adds a shape",
              "[ShapeManager][Lifecycle]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle handle{0, 0};

        shapeManager.AddShape(
            handle,
            std::make_unique<osseus::ShapeSphere>(5.0)
        );

        REQUIRE(shapeManager.GetShape(handle) != nullptr);
    }


    TEST_CASE("ShapeManager retrieves the correct sphere",
              "[ShapeManager][Handle][Invariant]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle handle{0, 0};

        shapeManager.AddShape(
            handle,
            std::make_unique<osseus::ShapeSphere>(5.0)
        );

        osseus::IShape* shape = shapeManager.GetShape(handle);

        REQUIRE(shape != nullptr);

        const auto* sphere =
            dynamic_cast<const osseus::ShapeSphere*>(shape);

        REQUIRE(sphere != nullptr);
        REQUIRE(sphere->GetRadius() == 5.0);
    }


    TEST_CASE("ShapeManager preserves polymorphic shape types",
              "[ShapeManager][Invariant]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle sphereHandle{0, 0};
        const osseus::Handle cubeHandle{1, 0};
        const osseus::Handle pointHandle{2, 0};

        shapeManager.AddShape(
            sphereHandle,
            std::make_unique<osseus::ShapeSphere>(5.0)
        );

        shapeManager.AddShape(
            cubeHandle,
            std::make_unique<osseus::ShapeCube>(3.0)
        );

        shapeManager.AddShape(
            pointHandle,
            std::make_unique<osseus::ShapePoint>()
        );

        REQUIRE(
            dynamic_cast<osseus::ShapeSphere*>(
                shapeManager.GetShape(sphereHandle)
            ) != nullptr
        );

        REQUIRE(
            dynamic_cast<osseus::ShapeCube*>(
                shapeManager.GetShape(cubeHandle)
            ) != nullptr
        );

        REQUIRE(
            dynamic_cast<osseus::ShapePoint*>(
                shapeManager.GetShape(pointHandle)
            ) != nullptr
        );
    }


    TEST_CASE("ShapeManager returns null for missing shape",
              "[ShapeManager][Handle]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle handle{42, 0};

        REQUIRE(shapeManager.GetShape(handle) == nullptr);
    }


    TEST_CASE("ShapeManager stores multiple shapes",
              "[ShapeManager][Stress][Invariant]") {
        osseus::ShapeManager shapeManager;

        constexpr std::size_t shapeCount = 100;

        for (std::uint32_t i = 0; i < shapeCount; ++i) {
            shapeManager.AddShape(
                osseus::Handle{i, 0},
                std::make_unique<osseus::ShapeSphere>(
                    static_cast<double>(i + 1)
                )
            );
        }

        REQUIRE(shapeManager.Data().size() == shapeCount);
        REQUIRE(shapeManager.Handles().size() == shapeCount);

        for (std::uint32_t i = 0; i < shapeCount; ++i) {
            const osseus::Handle handle{i, 0};

            osseus::IShape* shape = shapeManager.GetShape(handle);

            REQUIRE(shape != nullptr);

            const auto* sphere =
                dynamic_cast<const osseus::ShapeSphere*>(shape);

            REQUIRE(sphere != nullptr);
            REQUIRE(
                sphere->GetRadius()
                == static_cast<double>(i + 1)
            );
        }
    }


    TEST_CASE("ShapeManager removes a shape",
              "[ShapeManager][Lifecycle]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle handle{0, 0};

        shapeManager.AddShape(
            handle,
            std::make_unique<osseus::ShapeSphere>(5.0)
        );

        REQUIRE(shapeManager.GetShape(handle) != nullptr);

        shapeManager.RemoveShape(handle);

        REQUIRE(shapeManager.GetShape(handle) == nullptr);
        REQUIRE(shapeManager.Data().empty());
        REQUIRE(shapeManager.Handles().empty());
    }


    TEST_CASE("Removing one shape preserves other shapes",
              "[ShapeManager][Lifecycle][Invariant]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};
        const osseus::Handle third{2, 0};

        shapeManager.AddShape(
            first,
            std::make_unique<osseus::ShapeSphere>(1.0)
        );

        shapeManager.AddShape(
            second,
            std::make_unique<osseus::ShapeCube>(2.0)
        );

        shapeManager.AddShape(
            third,
            std::make_unique<osseus::ShapeSphere>(3.0)
        );

        shapeManager.RemoveShape(second);

        REQUIRE(shapeManager.GetShape(first) != nullptr);
        REQUIRE(shapeManager.GetShape(second) == nullptr);
        REQUIRE(shapeManager.GetShape(third) != nullptr);

        const auto* firstSphere =
            dynamic_cast<osseus::ShapeSphere*>(
                shapeManager.GetShape(first)
            );

        const auto* thirdSphere =
            dynamic_cast<osseus::ShapeSphere*>(
                shapeManager.GetShape(third)
            );

        REQUIRE(firstSphere != nullptr);
        REQUIRE(thirdSphere != nullptr);

        REQUIRE(firstSphere->GetRadius() == 1.0);
        REQUIRE(thirdSphere->GetRadius() == 3.0);
    }


    TEST_CASE("Removing the first shape preserves dense-set associations",
              "[ShapeManager][Lifecycle][Invariant][Regression]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle first{10, 0};
        const osseus::Handle second{20, 0};
        const osseus::Handle third{30, 0};

        shapeManager.AddShape(
            first,
            std::make_unique<osseus::ShapeSphere>(10.0)
        );

        shapeManager.AddShape(
            second,
            std::make_unique<osseus::ShapeCube>(20.0)
        );

        shapeManager.AddShape(
            third,
            std::make_unique<osseus::ShapeSphere>(30.0)
        );

        shapeManager.RemoveShape(first);

        REQUIRE(shapeManager.GetShape(first) == nullptr);

        auto* secondShape = shapeManager.GetShape(second);
        auto* thirdShape = shapeManager.GetShape(third);

        REQUIRE(secondShape != nullptr);
        REQUIRE(thirdShape != nullptr);

        const auto* secondCube =
            dynamic_cast<const osseus::ShapeCube*>(secondShape);

        const auto* thirdSphere =
            dynamic_cast<const osseus::ShapeSphere*>(thirdShape);

        REQUIRE(secondCube != nullptr);
        REQUIRE(thirdSphere != nullptr);

        REQUIRE(secondCube->GetHalfExtent() == 20.0);
        REQUIRE(thirdSphere->GetRadius() == 30.0);
    }


    TEST_CASE("Removing the middle shape preserves dense-set associations",
              "[ShapeManager][Lifecycle][Invariant][Regression]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle first{10, 0};
        const osseus::Handle second{20, 0};
        const osseus::Handle third{30, 0};
        const osseus::Handle fourth{40, 0};

        shapeManager.AddShape(
            first,
            std::make_unique<osseus::ShapeSphere>(10.0)
        );

        shapeManager.AddShape(
            second,
            std::make_unique<osseus::ShapeCube>(20.0)
        );

        shapeManager.AddShape(
            third,
            std::make_unique<osseus::ShapeSphere>(30.0)
        );

        shapeManager.AddShape(
            fourth,
            std::make_unique<osseus::ShapeCube>(40.0)
        );

        shapeManager.RemoveShape(second);

        REQUIRE(shapeManager.GetShape(first) != nullptr);
        REQUIRE(shapeManager.GetShape(second) == nullptr);
        REQUIRE(shapeManager.GetShape(third) != nullptr);
        REQUIRE(shapeManager.GetShape(fourth) != nullptr);

        const auto* firstSphere =
            dynamic_cast<const osseus::ShapeSphere*>(
                shapeManager.GetShape(first)
            );

        const auto* thirdSphere =
            dynamic_cast<const osseus::ShapeSphere*>(
                shapeManager.GetShape(third)
            );

        const auto* fourthCube =
            dynamic_cast<const osseus::ShapeCube*>(
                shapeManager.GetShape(fourth)
            );

        REQUIRE(firstSphere != nullptr);
        REQUIRE(thirdSphere != nullptr);
        REQUIRE(fourthCube != nullptr);

        REQUIRE(firstSphere->GetRadius() == 10.0);
        REQUIRE(thirdSphere->GetRadius() == 30.0);
        REQUIRE(fourthCube->GetHalfExtent() == 40.0);
    }


    TEST_CASE("Removing the last shape preserves previous shapes",
              "[ShapeManager][Lifecycle][Invariant]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};
        const osseus::Handle third{2, 0};

        shapeManager.AddShape(
            first,
            std::make_unique<osseus::ShapeSphere>(1.0)
        );

        shapeManager.AddShape(
            second,
            std::make_unique<osseus::ShapeCube>(2.0)
        );

        shapeManager.AddShape(
            third,
            std::make_unique<osseus::ShapeSphere>(3.0)
        );

        shapeManager.RemoveShape(third);

        REQUIRE(shapeManager.GetShape(first) != nullptr);
        REQUIRE(shapeManager.GetShape(second) != nullptr);
        REQUIRE(shapeManager.GetShape(third) == nullptr);

        REQUIRE(
            dynamic_cast<osseus::ShapeSphere*>(
                shapeManager.GetShape(first)
            ) != nullptr
        );

        REQUIRE(
            dynamic_cast<osseus::ShapeCube*>(
                shapeManager.GetShape(second)
            ) != nullptr
        );

        REQUIRE(shapeManager.Data().size() == 2);
        REQUIRE(shapeManager.Handles().size() == 2);
    }


    TEST_CASE("ShapeManager supports sparse handle indices",
              "[ShapeManager][Handle][Stress]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle first{2, 0};
        const osseus::Handle second{50, 0};
        const osseus::Handle third{1000, 0};

        shapeManager.AddShape(
            first,
            std::make_unique<osseus::ShapeSphere>(1.0)
        );

        shapeManager.AddShape(
            second,
            std::make_unique<osseus::ShapeCube>(2.0)
        );

        shapeManager.AddShape(
            third,
            std::make_unique<osseus::ShapeSphere>(3.0)
        );

        REQUIRE(shapeManager.GetShape(first) != nullptr);
        REQUIRE(shapeManager.GetShape(second) != nullptr);
        REQUIRE(shapeManager.GetShape(third) != nullptr);
    }


    TEST_CASE("ShapeManager preserves handle generations",
              "[ShapeManager][Handle][Invariant]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle handle{5, 7};

        shapeManager.AddShape(
            handle,
            std::make_unique<osseus::ShapeSphere>(5.0)
        );

        REQUIRE(shapeManager.GetShape(handle) != nullptr);
    }


    TEST_CASE("Stale handle cannot access replacement shape",
              "[ShapeManager][Handle][Lifecycle][Regression]") {
        osseus::ShapeManager shapeManager;

        const osseus::Handle original{5, 1};
        const osseus::Handle replacement{5, 2};

        shapeManager.AddShape(
            original,
            std::make_unique<osseus::ShapeSphere>(10.0)
        );

        shapeManager.RemoveShape(original);

        shapeManager.AddShape(
            replacement,
            std::make_unique<osseus::ShapeCube>(20.0)
        );

        REQUIRE(shapeManager.GetShape(original) == nullptr);

        auto* replacementShape =
            shapeManager.GetShape(replacement);

        REQUIRE(replacementShape != nullptr);

        const auto* replacementCube =
            dynamic_cast<const osseus::ShapeCube*>(replacementShape);

        REQUIRE(replacementCube != nullptr);
        REQUIRE(replacementCube->GetHalfExtent() == 20.0);
    }


    TEST_CASE("ShapeManager data and handles remain synchronized",
              "[ShapeManager][Invariant]") {
        osseus::ShapeManager shapeManager;

        for (std::uint32_t i = 0; i < 20; ++i) {
            shapeManager.AddShape(
                osseus::Handle{i, 0},
                std::make_unique<osseus::ShapeSphere>(
                    static_cast<double>(i + 1)
                )
            );
        }

        const auto& handles = shapeManager.Handles();
        const auto& data = shapeManager.Data();

        REQUIRE(handles.size() == data.size());

        for (std::size_t i = 0; i < handles.size(); ++i) {
            osseus::IShape* shape =
                shapeManager.GetShape(handles[i]);

            REQUIRE(shape != nullptr);

            const auto* sphere =
                dynamic_cast<const osseus::ShapeSphere*>(shape);

            REQUIRE(sphere != nullptr);

            REQUIRE(
                sphere->GetRadius()
                == static_cast<double>(i + 1)
            );
        }
    }


    TEST_CASE("ShapeManager remains consistent after repeated removal",
              "[ShapeManager][Lifecycle][Stress][Invariant]") {
        osseus::ShapeManager shapeManager;

        std::vector<osseus::Handle> handles;

        constexpr std::size_t shapeCount = 100;

        handles.reserve(shapeCount);

        for (std::uint32_t i = 0; i < shapeCount; ++i) {
            const osseus::Handle handle{i, 0};

            handles.push_back(handle);

            shapeManager.AddShape(
                handle,
                std::make_unique<osseus::ShapeSphere>(
                    static_cast<double>(i + 1)
                )
            );
        }

        for (std::size_t i = 0; i < handles.size(); i += 2) {
            shapeManager.RemoveShape(handles[i]);
        }

        REQUIRE(
            shapeManager.Data().size()
            == shapeCount / 2
        );

        REQUIRE(
            shapeManager.Handles().size()
            == shapeCount / 2
        );

        for (std::size_t i = 0; i < handles.size(); ++i) {
            if (i % 2 == 0) {
                REQUIRE(
                    shapeManager.GetShape(handles[i])
                    == nullptr
                );
            }
            else {
                auto* shape =
                    shapeManager.GetShape(handles[i]);

                REQUIRE(shape != nullptr);

                const auto* sphere =
                    dynamic_cast<const osseus::ShapeSphere*>(shape);

                REQUIRE(sphere != nullptr);

                REQUIRE(
                    sphere->GetRadius()
                    == static_cast<double>(i + 1)
                );
            }
        }
    }


    TEST_CASE("ShapeManager can remove all shapes",
              "[ShapeManager][Lifecycle][Stress]") {
        osseus::ShapeManager shapeManager;

        std::vector<osseus::Handle> handles;

        for (std::uint32_t i = 0; i < 50; ++i) {
            const osseus::Handle handle{i, 0};

            handles.push_back(handle);

            shapeManager.AddShape(
                handle,
                std::make_unique<osseus::ShapeSphere>(1.0)
            );
        }

        for (const osseus::Handle handle : handles) {
            shapeManager.RemoveShape(handle);
        }

        REQUIRE(shapeManager.Data().empty());
        REQUIRE(shapeManager.Handles().empty());

        for (const osseus::Handle handle : handles) {
            REQUIRE(shapeManager.GetShape(handle) == nullptr);
        }
    }

}
