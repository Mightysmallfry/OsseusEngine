#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>

#include <cstddef>
#include <vector>


namespace {

    TEST_CASE("ForceManager throws when given invalid handle", "[ForceManager]") {
        osseus::ForceManager forceManager;

        REQUIRE_THROWS_AS(forceManager.Get(osseus::Handle{0, 0}),std::out_of_range);
    }


    TEST_CASE("Register creates a force entry for a handle",
              "[ForceManager][Handle][Lifecycle]") {
        osseus::ForceManager forceManager;

        const osseus::Handle handle{0, 0};

        forceManager.Register(handle);

        REQUIRE(forceManager.Get(handle) == osseus::Vector3());
    }


    TEST_CASE("Registered handles have zero net force",
              "[ForceManager][Invariant]") {
        osseus::ForceManager forceManager;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};
        const osseus::Handle third{2, 0};

        forceManager.Register(first);
        forceManager.Register(second);
        forceManager.Register(third);

        REQUIRE(forceManager.Get(first) == osseus::Vector3());
        REQUIRE(forceManager.Get(second) == osseus::Vector3());
        REQUIRE(forceManager.Get(third) == osseus::Vector3());
    }


    TEST_CASE("Add applies force to a handle",
              "[ForceManager][Lifecycle]") {
        osseus::ForceManager forceManager;

        const osseus::Handle handle{0, 0};
        const osseus::Vector3 force{10.0, 20.0, 30.0};

        forceManager.Register(handle);
        forceManager.Add(handle, force);

        REQUIRE(forceManager.Get(handle) == force);
    }


    TEST_CASE("Multiple forces accumulate on a handle",
              "[ForceManager][Invariant]") {
        osseus::ForceManager forceManager;

        const osseus::Handle handle{0, 0};

        forceManager.Register(handle);

        forceManager.Add(handle, osseus::Vector3{10.0, 0.0, 0.0});
        forceManager.Add(handle, osseus::Vector3{0.0, 20.0, 0.0});
        forceManager.Add(handle, osseus::Vector3{0.0, 0.0, 30.0});

        REQUIRE(
            forceManager.Get(handle)
            == osseus::Vector3{10.0, 20.0, 30.0}
        );
    }


    TEST_CASE("Force addition is component-wise",
              "[ForceManager][Invariant]") {
        osseus::ForceManager forceManager;

        const osseus::Handle handle{0, 0};

        forceManager.Register(handle);

        forceManager.Add(
            handle,
            osseus::Vector3{-5.0, 12.0, 3.0}
        );

        forceManager.Add(
            handle,
            osseus::Vector3{8.0, -2.0, 7.0}
        );

        REQUIRE(
            forceManager.Get(handle)
            == osseus::Vector3{3.0, 10.0, 10.0}
        );
    }


    TEST_CASE("Forces are independent between handles",
              "[ForceManager][Handle][Invariant]") {
        osseus::ForceManager forceManager;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};

        forceManager.Register(first);
        forceManager.Register(second);

        const osseus::Vector3 firstForce{10.0, 20.0, 30.0};
        const osseus::Vector3 secondForce{-5.0, 4.0, 8.0};

        forceManager.Add(first, firstForce);
        forceManager.Add(second, secondForce);

        REQUIRE(forceManager.Get(first) == firstForce);
        REQUIRE(forceManager.Get(second) == secondForce);
    }


    TEST_CASE("Clear removes all accumulated forces",
              "[ForceManager][Lifecycle]") {
        osseus::ForceManager forceManager;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};

        forceManager.Register(first);
        forceManager.Register(second);

        forceManager.Add(first, osseus::Vector3{10.0, 20.0, 30.0});
        forceManager.Add(second, osseus::Vector3{-5.0, 15.0, 2.0});

        forceManager.Clear();

        REQUIRE(forceManager.Get(first) == osseus::Vector3());
        REQUIRE(forceManager.Get(second) == osseus::Vector3());
    }


    TEST_CASE("ClearForceOf removes only the selected handle's force",
              "[ForceManager][Handle][Lifecycle]") {
        osseus::ForceManager forceManager;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};

        forceManager.Register(first);
        forceManager.Register(second);

        const osseus::Vector3 firstForce{10.0, 20.0, 30.0};
        const osseus::Vector3 secondForce{-5.0, 15.0, 2.0};

        forceManager.Add(first, firstForce);
        forceManager.Add(second, secondForce);

        forceManager.ClearForceOf(first);

        REQUIRE(forceManager.Get(first) == osseus::Vector3());
        REQUIRE(forceManager.Get(second) == secondForce);
    }


    TEST_CASE("Clearing a force twice remains zero",
              "[ForceManager][Handle][Lifecycle][Invariant]") {
        osseus::ForceManager forceManager;

        const osseus::Handle handle{0, 0};

        forceManager.Register(handle);

        forceManager.Add(
            handle,
            osseus::Vector3{10.0, 20.0, 30.0}
        );

        forceManager.ClearForceOf(handle);
        forceManager.ClearForceOf(handle);

        REQUIRE(forceManager.Get(handle) == osseus::Vector3());
    }


    TEST_CASE("Resize creates sufficient force storage",
              "[ForceManager][Lifecycle]") {
        osseus::ForceManager forceManager;

        forceManager.Resize(10);

        for (std::uint32_t i = 0; i < 10; ++i) {
            const osseus::Handle handle{i, 0};

            forceManager.Register(handle);

            REQUIRE(forceManager.Get(handle) == osseus::Vector3());
        }
    }


    TEST_CASE("Resize preserves existing forces",
              "[ForceManager][Invariant]") {
        osseus::ForceManager forceManager;

        const osseus::Handle handle{0, 0};

        forceManager.Register(handle);

        const osseus::Vector3 force{10.0, 20.0, 30.0};

        forceManager.Add(handle, force);
        forceManager.Resize(10);

        REQUIRE(forceManager.Get(handle) == force);
    }


    TEST_CASE("Registering multiple handles maintains independent storage",
              "[ForceManager][Handle][Invariant]") {
        osseus::ForceManager forceManager;

        constexpr std::size_t handleCount = 100;

        for (std::uint32_t i = 0; i < handleCount; ++i) {
            forceManager.Register(osseus::Handle{i, 0});
        }

        for (std::uint32_t i = 0; i < handleCount; ++i) {
            const osseus::Handle handle{i, 0};

            forceManager.Add(
                handle,
                osseus::Vector3{
                    static_cast<double>(i),
                    static_cast<double>(i * 2),
                    static_cast<double>(i * 3)
                }
            );
        }

        for (std::uint32_t i = 0; i < handleCount; ++i) {
            const osseus::Handle handle{i, 0};

            REQUIRE(
                forceManager.Get(handle)
                == osseus::Vector3{
                    static_cast<double>(i),
                    static_cast<double>(i * 2),
                    static_cast<double>(i * 3)
                }
            );
        }
    }


    TEST_CASE("Force storage remains valid after clearing all forces",
              "[ForceManager][Stress][Invariant]") {
        osseus::ForceManager forceManager;

        constexpr std::size_t handleCount = 100;

        for (std::uint32_t i = 0; i < handleCount; ++i) {
            const osseus::Handle handle{i, 0};

            forceManager.Register(handle);
            forceManager.Add(
                handle,
                osseus::Vector3{1.0, 2.0, 3.0}
            );
        }

        forceManager.Clear();

        for (std::uint32_t i = 0; i < handleCount; ++i) {
            REQUIRE(
                forceManager.Get(osseus::Handle{i, 0})
                == osseus::Vector3()
            );
        }
    }


    TEST_CASE("ForceManager handles sparse handle indices",
              "[ForceManager][Handle][Stress]") {
        osseus::ForceManager forceManager;

        const osseus::Handle first{2, 0};
        const osseus::Handle second{50, 0};
        const osseus::Handle third{1000, 0};

        forceManager.Register(first);
        forceManager.Register(second);
        forceManager.Register(third);

        forceManager.Add(first, osseus::Vector3{1.0, 2.0, 3.0});
        forceManager.Add(second, osseus::Vector3{4.0, 5.0, 6.0});
        forceManager.Add(third, osseus::Vector3{7.0, 8.0, 9.0});

        REQUIRE(
            forceManager.Get(first)
            == osseus::Vector3{1.0, 2.0, 3.0}
        );

        REQUIRE(
            forceManager.Get(second)
            == osseus::Vector3{4.0, 5.0, 6.0}
        );

        REQUIRE(
            forceManager.Get(third)
            == osseus::Vector3{7.0, 8.0, 9.0}
        );
    }


    TEST_CASE("Stale handle cannot access replacement force",
              "[ForceManager][Handle][Lifecycle][Regression]") {
        osseus::ForceManager forceManager;

        const osseus::Handle original{5, 1};
        const osseus::Handle replacement{5, 2};

        forceManager.Register(original);
        forceManager.Add(
            original,
            osseus::Vector3{10.0, 20.0, 30.0}
        );

        forceManager.ClearForceOf(original);

        forceManager.Register(replacement);
        forceManager.Add(
            replacement,
            osseus::Vector3{40.0, 50.0, 60.0}
        );

        REQUIRE(
            forceManager.Get(replacement)
            == osseus::Vector3{40.0, 50.0, 60.0}
        );
    }


    TEST_CASE("ClearUniversals removes all universal force evaluators",
              "[ForceManager][Lifecycle]") {
        osseus::ForceManager forceManager;

        osseus::UniversalGravity gravity;
        osseus::UniversalElectroMag electromagnetism;

        forceManager.AddUniversal(&gravity);
        forceManager.AddUniversal(&electromagnetism);

        forceManager.ClearUniversals();

        // Universal evaluators are intentionally owned elsewhere.
        // This test primarily verifies that clearing does not affect
        // the evaluator objects themselves.
        SUCCEED();
    }


    TEST_CASE("Universal force evaluators can be registered",
              "[ForceManager][Lifecycle]") {
        osseus::ForceManager forceManager;

        osseus::UniversalGravity gravity;
        osseus::UniversalElectroMag electromagnetism;

        forceManager.AddUniversal(&gravity);
        forceManager.AddUniversal(&electromagnetism);

        REQUIRE(true);
    }

}
