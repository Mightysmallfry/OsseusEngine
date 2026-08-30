#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>

#include <cstddef>
#include <vector>


namespace {

    TEST_CASE("New handles are valid", "[Registry][Handle][Lifecycle]") {
        osseus::Registry registry;

        const osseus::Handle handle = registry.CreateHandle();

        REQUIRE(registry.IsValid(handle));
    }


    TEST_CASE("Created handles have unique indices",
              "[Registry][Handle][Invariant]") {
        osseus::Registry registry;

        const osseus::Handle first = registry.CreateHandle();
        const osseus::Handle second = registry.CreateHandle();
        const osseus::Handle third = registry.CreateHandle();

        REQUIRE(first.index != second.index);
        REQUIRE(first.index != third.index);
        REQUIRE(second.index != third.index);
    }


    TEST_CASE("Created handles have valid generations",
              "[Registry][Handle][Invariant]") {
        osseus::Registry registry;

        const osseus::Handle first = registry.CreateHandle();
        const osseus::Handle second = registry.CreateHandle();

        REQUIRE(registry.IsValid(first));
        REQUIRE(registry.IsValid(second));
    }


    TEST_CASE("Destroying a handle invalidates it",
              "[Registry][Handle][Lifecycle]") {
        osseus::Registry registry;

        const osseus::Handle handle = registry.CreateHandle();

        REQUIRE(registry.IsValid(handle));

        registry.Destroy(handle);

        REQUIRE_FALSE(registry.IsValid(handle));
    }


    TEST_CASE("Destroying one handle does not invalidate other handles",
              "[Registry][Handle][Lifecycle][Invariant]") {
        osseus::Registry registry;

        const osseus::Handle first = registry.CreateHandle();
        const osseus::Handle second = registry.CreateHandle();
        const osseus::Handle third = registry.CreateHandle();

        registry.Destroy(second);

        REQUIRE(registry.IsValid(first));
        REQUIRE_FALSE(registry.IsValid(second));
        REQUIRE(registry.IsValid(third));
    }


    TEST_CASE("Destroyed handle index is reused",
              "[Registry][Handle][Lifecycle]") {
        osseus::Registry registry;

        const osseus::Handle first = registry.CreateHandle();

        registry.Destroy(first);

        const osseus::Handle replacement = registry.CreateHandle();

        REQUIRE(replacement.index == first.index);
    }


    TEST_CASE("Reused handle receives a new generation",
              "[Registry][Handle][Lifecycle][Invariant]") {
        osseus::Registry registry;

        const osseus::Handle original = registry.CreateHandle();

        registry.Destroy(original);

        const osseus::Handle replacement = registry.CreateHandle();

        REQUIRE(replacement.index == original.index);
        REQUIRE(replacement.generation != original.generation);
    }


    TEST_CASE("Stale handle remains invalid after index reuse",
              "[Registry][Handle][Lifecycle][Regression]") {
        osseus::Registry registry;

        const osseus::Handle original = registry.CreateHandle();

        registry.Destroy(original);

        const osseus::Handle replacement = registry.CreateHandle();

        REQUIRE_FALSE(registry.IsValid(original));
        REQUIRE(registry.IsValid(replacement));
    }


    TEST_CASE("Destroying a stale handle does not destroy replacement",
              "[Registry][Handle][Lifecycle][Regression]") {
        osseus::Registry registry;

        const osseus::Handle original = registry.CreateHandle();

        registry.Destroy(original);

        const osseus::Handle replacement = registry.CreateHandle();

        registry.Destroy(original);

        REQUIRE_FALSE(registry.IsValid(original));
        REQUIRE(registry.IsValid(replacement));
    }


    TEST_CASE("Multiple index reuses receive new generations",
              "[Registry][Handle][Lifecycle][Invariant]") {
        osseus::Registry registry;

        const osseus::Handle first = registry.CreateHandle();

        registry.Destroy(first);

        const osseus::Handle second = registry.CreateHandle();

        registry.Destroy(second);

        const osseus::Handle third = registry.CreateHandle();

        REQUIRE(first.index == second.index);
        REQUIRE(second.index == third.index);

        REQUIRE(first.generation != second.generation);
        REQUIRE(second.generation != third.generation);
        REQUIRE(first.generation != third.generation);

        REQUIRE_FALSE(registry.IsValid(first));
        REQUIRE_FALSE(registry.IsValid(second));
        REQUIRE(registry.IsValid(third));
    }


    TEST_CASE("Destroying a handle twice does not invalidate another handle",
              "[Registry][Handle][Lifecycle][Regression]") {
        osseus::Registry registry;

        const osseus::Handle first = registry.CreateHandle();
        const osseus::Handle second = registry.CreateHandle();

        registry.Destroy(first);
        registry.Destroy(first);

        REQUIRE_FALSE(registry.IsValid(first));
        REQUIRE(registry.IsValid(second));
    }


    TEST_CASE("Handle validity is independent between generations",
              "[Registry][Handle][Invariant][Regression]") {
        osseus::Registry registry;

        const osseus::Handle original = registry.CreateHandle();

        registry.Destroy(original);

        const osseus::Handle replacement = registry.CreateHandle();

        REQUIRE(original.index == replacement.index);
        REQUIRE(original.generation != replacement.generation);

        REQUIRE_FALSE(registry.IsValid(original));
        REQUIRE(registry.IsValid(replacement));
    }


    TEST_CASE("Multiple live handles remain valid",
              "[Registry][Handle][Stress][Invariant]") {
        osseus::Registry registry;

        constexpr std::size_t handleCount = 100;

        std::vector<osseus::Handle> handles;
        handles.reserve(handleCount);

        for (std::size_t i = 0; i < handleCount; ++i) {
            handles.push_back(registry.CreateHandle());
        }

        for (const osseus::Handle handle : handles) {
            REQUIRE(registry.IsValid(handle));
        }
    }


    TEST_CASE("Destroying multiple handles preserves remaining handles",
              "[Registry][Handle][Stress][Lifecycle][Invariant]") {
        osseus::Registry registry;

        std::vector<osseus::Handle> handles;

        for (int i = 0; i < 10; ++i) {
            handles.push_back(registry.CreateHandle());
        }

        registry.Destroy(handles[2]);
        registry.Destroy(handles[5]);
        registry.Destroy(handles[8]);

        for (std::size_t i = 0; i < handles.size(); ++i) {
            if (i == 2 || i == 5 || i == 8) {
                REQUIRE_FALSE(registry.IsValid(handles[i]));
            }
            else {
                REQUIRE(registry.IsValid(handles[i]));
            }
        }
    }


    TEST_CASE("New handles after destruction are valid",
              "[Registry][Handle][Lifecycle][Invariant]") {
        osseus::Registry registry;

        const osseus::Handle first = registry.CreateHandle();
        const osseus::Handle second = registry.CreateHandle();

        registry.Destroy(first);
        registry.Destroy(second);

        const osseus::Handle replacementFirst = registry.CreateHandle();
        const osseus::Handle replacementSecond = registry.CreateHandle();

        REQUIRE(registry.IsValid(replacementFirst));
        REQUIRE(registry.IsValid(replacementSecond));
    }

}
