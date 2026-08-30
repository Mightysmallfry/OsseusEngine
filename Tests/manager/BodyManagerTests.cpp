#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>

#include <cstddef>
#include <vector>


namespace {

    osseus::BodyData MakeBody(
        double x,
        double y,
        double z,
        double mass
    ) {
        osseus::BodyData body;

        body.position = osseus::Vector3{x, y, z};
        body.velocity = osseus::Vector3{};
        body.mass = mass;
        body.invMass = mass != 0.0 ? 1.0 / mass : 0.0;
        body.charge = 0.0;

        return body;
    }


    TEST_CASE("BodyManager starts empty", "[BodyManager]") {
        osseus::BodyManager bodyManager;

        REQUIRE(bodyManager.Data().empty());
        REQUIRE(bodyManager.Handles().empty());
    }


    TEST_CASE("BodyManager registers a handle",
              "[BodyManager][Handle][Lifecycle]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle handle{0, 0};

        bodyManager.Register(handle);

        REQUIRE(bodyManager.GetBody(handle) != nullptr);
    }


    TEST_CASE("BodyManager adds a body",
              "[BodyManager][Lifecycle]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle handle{0, 0};
        const osseus::BodyData body = MakeBody(
            10.0,
            20.0,
            30.0,
            5.0
        );

        bodyManager.AddBody(handle, body);

        REQUIRE(bodyManager.GetBody(handle) != nullptr);
        REQUIRE(bodyManager.GetBody(handle)->position == body.position);
        REQUIRE(bodyManager.GetBody(handle)->velocity == body.velocity);
        REQUIRE(bodyManager.GetBody(handle)->mass == body.mass);
        REQUIRE(bodyManager.GetBody(handle)->invMass == body.invMass);
        REQUIRE(bodyManager.GetBody(handle)->charge == body.charge);
    }


    TEST_CASE("BodyManager retrieves the correct body",
              "[BodyManager][Handle][Invariant]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};

        bodyManager.AddBody(
            first,
            MakeBody(10.0, 0.0, 0.0, 2.0)
        );

        bodyManager.AddBody(
            second,
            MakeBody(20.0, 0.0, 0.0, 4.0)
        );

        REQUIRE(bodyManager.GetBody(first)->position.x == 10.0);
        REQUIRE(bodyManager.GetBody(first)->mass == 2.0);

        REQUIRE(bodyManager.GetBody(second)->position.x == 20.0);
        REQUIRE(bodyManager.GetBody(second)->mass == 4.0);
    }


    TEST_CASE("BodyManager returns null for missing body",
              "[BodyManager][Handle]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle handle{42, 0};

        REQUIRE(bodyManager.GetBody(handle) == nullptr);
    }


    TEST_CASE("BodyManager stores multiple bodies",
              "[BodyManager][Stress][Invariant]") {
        osseus::BodyManager bodyManager;

        constexpr std::size_t bodyCount = 100;

        for (std::uint32_t i = 0; i < bodyCount; ++i) {
            bodyManager.AddBody(
                osseus::Handle{i, 0},
                MakeBody(
                    static_cast<double>(i),
                    0.0,
                    0.0,
                    static_cast<double>(i + 1)
                )
            );
        }

        REQUIRE(bodyManager.Data().size() == bodyCount);
        REQUIRE(bodyManager.Handles().size() == bodyCount);

        for (std::uint32_t i = 0; i < bodyCount; ++i) {
            const osseus::Handle handle{i, 0};

            REQUIRE(bodyManager.GetBody(handle) != nullptr);
            REQUIRE(bodyManager.GetBody(handle)->position.x == static_cast<double>(i));
            REQUIRE(bodyManager.GetBody(handle)->mass == static_cast<double>(i + 1));
        }
    }


    TEST_CASE("BodyManager removes a body",
              "[BodyManager][Lifecycle]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle handle{0, 0};

        bodyManager.AddBody(
            handle,
            MakeBody(10.0, 20.0, 30.0, 5.0)
        );

        REQUIRE(bodyManager.GetBody(handle) != nullptr);

        bodyManager.RemoveBody(handle);

        REQUIRE(bodyManager.GetBody(handle) == nullptr);
        REQUIRE(bodyManager.Data().empty());
        REQUIRE(bodyManager.Handles().empty());
    }


    TEST_CASE("Removing one body preserves other bodies",
              "[BodyManager][Lifecycle][Invariant]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};
        const osseus::Handle third{2, 0};

        bodyManager.AddBody(
            first,
            MakeBody(10.0, 0.0, 0.0, 1.0)
        );

        bodyManager.AddBody(
            second,
            MakeBody(20.0, 0.0, 0.0, 2.0)
        );

        bodyManager.AddBody(
            third,
            MakeBody(30.0, 0.0, 0.0, 3.0)
        );

        bodyManager.RemoveBody(second);

        REQUIRE(bodyManager.GetBody(first) != nullptr);
        REQUIRE(bodyManager.GetBody(second) == nullptr);
        REQUIRE(bodyManager.GetBody(third) != nullptr);

        REQUIRE(bodyManager.GetBody(first)->mass == 1.0);
        REQUIRE(bodyManager.GetBody(third)->mass == 3.0);

        REQUIRE(bodyManager.Data().size() == 2);
        REQUIRE(bodyManager.Handles().size() == 2);
    }


    TEST_CASE("Removing the first body preserves dense-set associations",
              "[BodyManager][Lifecycle][Invariant][Regression]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle first{10, 0};
        const osseus::Handle second{20, 0};
        const osseus::Handle third{30, 0};

        bodyManager.AddBody(
            first,
            MakeBody(100.0, 0.0, 0.0, 10.0)
        );

        bodyManager.AddBody(
            second,
            MakeBody(200.0, 0.0, 0.0, 20.0)
        );

        bodyManager.AddBody(
            third,
            MakeBody(300.0, 0.0, 0.0, 30.0)
        );

        bodyManager.RemoveBody(first);

        REQUIRE(bodyManager.GetBody(first) == nullptr);

        REQUIRE(bodyManager.GetBody(second) != nullptr);
        REQUIRE(bodyManager.GetBody(third) != nullptr);

        REQUIRE(bodyManager.GetBody(second)->position.x == 200.0);
        REQUIRE(bodyManager.GetBody(second)->mass == 20.0);

        REQUIRE(bodyManager.GetBody(third)->position.x == 300.0);
        REQUIRE(bodyManager.GetBody(third)->mass == 30.0);
    }


    TEST_CASE("Removing the middle body preserves dense-set associations",
              "[BodyManager][Lifecycle][Invariant][Regression]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle first{10, 0};
        const osseus::Handle second{20, 0};
        const osseus::Handle third{30, 0};
        const osseus::Handle fourth{40, 0};

        bodyManager.AddBody(
            first,
            MakeBody(100.0, 0.0, 0.0, 10.0)
        );

        bodyManager.AddBody(
            second,
            MakeBody(200.0, 0.0, 0.0, 20.0)
        );

        bodyManager.AddBody(
            third,
            MakeBody(300.0, 0.0, 0.0, 30.0)
        );

        bodyManager.AddBody(
            fourth,
            MakeBody(400.0, 0.0, 0.0, 40.0)
        );

        bodyManager.RemoveBody(second);

        REQUIRE(bodyManager.GetBody(first)->mass == 10.0);
        REQUIRE(bodyManager.GetBody(third)->mass == 30.0);
        REQUIRE(bodyManager.GetBody(fourth)->mass == 40.0);

        REQUIRE(bodyManager.GetBody(second) == nullptr);
    }


    TEST_CASE("Removing the last body preserves previous bodies",
              "[BodyManager][Lifecycle][Invariant]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};
        const osseus::Handle third{2, 0};

        bodyManager.AddBody(
            first,
            MakeBody(10.0, 0.0, 0.0, 1.0)
        );

        bodyManager.AddBody(
            second,
            MakeBody(20.0, 0.0, 0.0, 2.0)
        );

        bodyManager.AddBody(
            third,
            MakeBody(30.0, 0.0, 0.0, 3.0)
        );

        bodyManager.RemoveBody(third);

        REQUIRE(bodyManager.GetBody(first)->mass == 1.0);
        REQUIRE(bodyManager.GetBody(second)->mass == 2.0);
        REQUIRE(bodyManager.GetBody(third) == nullptr);

        REQUIRE(bodyManager.Data().size() == 2);
    }


    TEST_CASE("BodyManager supports sparse handle indices",
              "[BodyManager][Handle][Stress]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle first{2, 0};
        const osseus::Handle second{50, 0};
        const osseus::Handle third{1000, 0};

        bodyManager.AddBody(
            first,
            MakeBody(10.0, 0.0, 0.0, 1.0)
        );

        bodyManager.AddBody(
            second,
            MakeBody(20.0, 0.0, 0.0, 2.0)
        );

        bodyManager.AddBody(
            third,
            MakeBody(30.0, 0.0, 0.0, 3.0)
        );

        REQUIRE(bodyManager.GetBody(first)->mass == 1.0);
        REQUIRE(bodyManager.GetBody(second)->mass == 2.0);
        REQUIRE(bodyManager.GetBody(third)->mass == 3.0);
    }


    TEST_CASE("BodyManager preserves handle generations",
              "[BodyManager][Handle][Invariant]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle handle{5, 7};

        bodyManager.AddBody(
            handle,
            MakeBody(10.0, 20.0, 30.0, 5.0)
        );

        REQUIRE(bodyManager.GetBody(handle) != nullptr);
        REQUIRE(bodyManager.GetBody(handle)->mass == 5.0);
    }


    TEST_CASE("Stale handle cannot access replacement body",
              "[BodyManager][Handle][Lifecycle][Regression]") {
        osseus::BodyManager bodyManager;

        const osseus::Handle original{5, 1};
        const osseus::Handle replacement{5, 2};

        bodyManager.AddBody(
            original,
            MakeBody(10.0, 0.0, 0.0, 10.0)
        );

        bodyManager.RemoveBody(original);

        bodyManager.AddBody(
            replacement,
            MakeBody(20.0, 0.0, 0.0, 20.0)
        );

        REQUIRE(bodyManager.GetBody(original) == nullptr);
        REQUIRE(bodyManager.GetBody(replacement) != nullptr);
        REQUIRE(bodyManager.GetBody(replacement)->mass == 20.0);
    }


    TEST_CASE("BodyManager data and handles remain synchronized",
              "[BodyManager][Invariant]") {
        osseus::BodyManager bodyManager;

        for (std::uint32_t i = 0; i < 20; ++i) {
            bodyManager.AddBody(
                osseus::Handle{i, 0},
                MakeBody(
                    static_cast<double>(i),
                    0.0,
                    0.0,
                    static_cast<double>(i + 1)
                )
            );
        }

        const auto& handles = bodyManager.Handles();
        const auto& data = bodyManager.Data();

        REQUIRE(handles.size() == data.size());

        for (std::size_t i = 0; i < handles.size(); ++i) {
            osseus::BodyData* body = bodyManager.GetBody(handles[i]);

            REQUIRE(body != nullptr);
            REQUIRE(body->position.x == data[i].position.x);
            REQUIRE(body->mass == data[i].mass);
        }
    }


    TEST_CASE("BodyManager remains consistent after repeated removal",
              "[BodyManager][Lifecycle][Stress][Invariant]") {
        osseus::BodyManager bodyManager;

        std::vector<osseus::Handle> handles;

        constexpr std::size_t bodyCount = 100;

        handles.reserve(bodyCount);

        for (std::uint32_t i = 0; i < bodyCount; ++i) {
            const osseus::Handle handle{i, 0};

            handles.push_back(handle);

            bodyManager.AddBody(
                handle,
                MakeBody(
                    static_cast<double>(i),
                    0.0,
                    0.0,
                    static_cast<double>(i + 1)
                )
            );
        }

        for (std::size_t i = 0; i < handles.size(); i += 2) {
            bodyManager.RemoveBody(handles[i]);
        }

        REQUIRE(bodyManager.Data().size() == bodyCount / 2);
        REQUIRE(bodyManager.Handles().size() == bodyCount / 2);

        for (std::size_t i = 0; i < handles.size(); ++i) {
            if (i % 2 == 0) {
                REQUIRE(bodyManager.GetBody(handles[i]) == nullptr);
            }
            else {
                REQUIRE(bodyManager.GetBody(handles[i]) != nullptr);
                REQUIRE(
                    bodyManager.GetBody(handles[i])->mass
                    == static_cast<double>(i + 1)
                );
            }
        }
    }


    TEST_CASE("BodyManager can remove all bodies",
              "[BodyManager][Lifecycle][Stress]") {
        osseus::BodyManager bodyManager;

        std::vector<osseus::Handle> handles;

        for (std::uint32_t i = 0; i < 50; ++i) {
            const osseus::Handle handle{i, 0};

            handles.push_back(handle);

            bodyManager.AddBody(
                handle,
                MakeBody(
                    static_cast<double>(i),
                    0.0,
                    0.0,
                    1.0
                )
            );
        }

        for (const osseus::Handle handle : handles) {
            bodyManager.RemoveBody(handle);
        }

        REQUIRE(bodyManager.Data().empty());
        REQUIRE(bodyManager.Handles().empty());

        for (const osseus::Handle handle : handles) {
            REQUIRE(bodyManager.GetBody(handle) == nullptr);
        }
    }

}
