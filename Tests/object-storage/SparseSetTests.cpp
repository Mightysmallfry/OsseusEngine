#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>

#include <cstddef>
#include <vector>

namespace {

    struct TestData {
        int value = 0;
    };


    TEST_CASE("SparseSet starts empty", "[SparseSet]") {
        osseus::SparseSet<TestData> set;

        REQUIRE(set.size() == 0);
    }


    TEST_CASE("SparseSet inserts an element", "[SparseSet]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle handle{0, 0};

        set.Insert(handle, TestData{42});

        REQUIRE(set.size() == 1);
        REQUIRE(set.Contains(handle));
    }


    TEST_CASE("SparseSet retrieves inserted element", "[SparseSet]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle handle{0, 0};

        set.Insert(handle, TestData{42});

        TestData* data = set.Get(handle);

        REQUIRE(data != nullptr);
        REQUIRE(data->value == 42);
    }


    TEST_CASE("SparseSet returns null for missing handle", "[SparseSet]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle handle{0, 0};

        REQUIRE_FALSE(set.Contains(handle));
        REQUIRE(set.Get(handle) == nullptr);
    }


    TEST_CASE("SparseSet stores multiple elements", "[SparseSet]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};
        const osseus::Handle third{2, 0};

        set.Insert(first, TestData{10});
        set.Insert(second, TestData{20});
        set.Insert(third, TestData{30});

        REQUIRE(set.size() == 3);

        REQUIRE(set.Contains(first));
        REQUIRE(set.Contains(second));
        REQUIRE(set.Contains(third));

        REQUIRE(set.Get(first)->value == 10);
        REQUIRE(set.Get(second)->value == 20);
        REQUIRE(set.Get(third)->value == 30);
    }


    TEST_CASE("SparseSet supports sparse handle indices", "[SparseSet]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle first{2, 0};
        const osseus::Handle second{50, 0};
        const osseus::Handle third{1000, 0};

        set.Insert(first, TestData{10});
        set.Insert(second, TestData{20});
        set.Insert(third, TestData{30});

        REQUIRE(set.size() == 3);

        REQUIRE(set.Get(first)->value == 10);
        REQUIRE(set.Get(second)->value == 20);
        REQUIRE(set.Get(third)->value == 30);
    }


    TEST_CASE("SparseSet preserves handle generations", "[SparseSet][Handle]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle handle{5, 7};

        set.Insert(handle, TestData{42});

        REQUIRE(set.Contains(handle));
        REQUIRE(set.Get(handle)->value == 42);
    }


    TEST_CASE("SparseSet rejects stale generation", "[SparseSet][Handle]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle current{5, 2};
        const osseus::Handle stale{5, 1};

        set.Insert(current, TestData{42});

        REQUIRE(set.Contains(current));
        REQUIRE_FALSE(set.Contains(stale));
        REQUIRE(set.Get(stale) == nullptr);
    }


    TEST_CASE("SparseSet removes an element", "[SparseSet][Lifecycle]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle handle{0, 0};

        set.Insert(handle, TestData{42});

        REQUIRE(set.Contains(handle));

        set.Remove(handle);

        REQUIRE(set.size() == 0);
        REQUIRE_FALSE(set.Contains(handle));
        REQUIRE(set.Get(handle) == nullptr);
    }


    TEST_CASE("SparseSet removes first element", "[SparseSet][Lifecycle]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};
        const osseus::Handle third{2, 0};

        set.Insert(first, TestData{10});
        set.Insert(second, TestData{20});
        set.Insert(third, TestData{30});

        set.Remove(first);

        REQUIRE(set.size() == 2);

        REQUIRE_FALSE(set.Contains(first));
        REQUIRE(set.Contains(second));
        REQUIRE(set.Contains(third));

        REQUIRE(set.Get(second)->value == 20);
        REQUIRE(set.Get(third)->value == 30);
    }


    TEST_CASE("SparseSet removes middle element", "[SparseSet][Lifecycle]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};
        const osseus::Handle third{2, 0};

        set.Insert(first, TestData{10});
        set.Insert(second, TestData{20});
        set.Insert(third, TestData{30});

        set.Remove(second);

        REQUIRE(set.size() == 2);

        REQUIRE(set.Contains(first));
        REQUIRE_FALSE(set.Contains(second));
        REQUIRE(set.Contains(third));

        REQUIRE(set.Get(first)->value == 10);
        REQUIRE(set.Get(third)->value == 30);
    }


    TEST_CASE("SparseSet removes last element", "[SparseSet][Lifecycle]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};
        const osseus::Handle third{2, 0};

        set.Insert(first, TestData{10});
        set.Insert(second, TestData{20});
        set.Insert(third, TestData{30});

        set.Remove(third);

        REQUIRE(set.size() == 2);

        REQUIRE(set.Contains(first));
        REQUIRE(set.Contains(second));
        REQUIRE_FALSE(set.Contains(third));

        REQUIRE(set.Get(first)->value == 10);
        REQUIRE(set.Get(second)->value == 20);
    }


    TEST_CASE("SparseSet preserves remaining elements after swap and pop",
              "[SparseSet][Lifecycle]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle first{10, 0};
        const osseus::Handle second{20, 0};
        const osseus::Handle third{30, 0};
        const osseus::Handle fourth{40, 0};

        set.Insert(first, TestData{10});
        set.Insert(second, TestData{20});
        set.Insert(third, TestData{30});
        set.Insert(fourth, TestData{40});

        set.Remove(second);

        REQUIRE(set.size() == 3);

        REQUIRE(set.Get(first)->value == 10);
        REQUIRE(set.Get(third)->value == 30);
        REQUIRE(set.Get(fourth)->value == 40);

        REQUIRE_FALSE(set.Contains(second));
    }


    TEST_CASE("SparseSet can remove all elements", "[SparseSet][Lifecycle]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle first{0, 0};
        const osseus::Handle second{1, 0};
        const osseus::Handle third{2, 0};

        set.Insert(first, TestData{10});
        set.Insert(second, TestData{20});
        set.Insert(third, TestData{30});

        set.Remove(first);
        set.Remove(second);
        set.Remove(third);

        REQUIRE(set.size() == 0);

        REQUIRE_FALSE(set.Contains(first));
        REQUIRE_FALSE(set.Contains(second));
        REQUIRE_FALSE(set.Contains(third));
    }


    TEST_CASE("SparseSet supports remove and reinsert",
              "[SparseSet][Lifecycle]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle handle{5, 0};

        set.Insert(handle, TestData{10});
        set.Remove(handle);

        REQUIRE_FALSE(set.Contains(handle));

        set.Insert(handle, TestData{20});

        REQUIRE(set.size() == 1);
        REQUIRE(set.Contains(handle));
        REQUIRE(set.Get(handle)->value == 20);
    }


    TEST_CASE("SparseSet handles replacement after removal",
              "[SparseSet][Lifecycle][Handle]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle original{5, 1};
        const osseus::Handle replacement{5, 2};

        set.Insert(original, TestData{10});
        set.Remove(original);

        set.Insert(replacement, TestData{20});

        REQUIRE_FALSE(set.Contains(original));
        REQUIRE(set.Contains(replacement));

        REQUIRE(set.Get(original) == nullptr);
        REQUIRE(set.Get(replacement) != nullptr);
        REQUIRE(set.Get(replacement)->value == 20);
    }


    TEST_CASE("SparseSet data and handles remain synchronized",
              "[SparseSet][Invariant]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle first{10, 0};
        const osseus::Handle second{20, 0};
        const osseus::Handle third{30, 0};

        set.Insert(first, TestData{100});
        set.Insert(second, TestData{200});
        set.Insert(third, TestData{300});

        const auto& handles = set.Handles();
        const auto& data = set.Data();

        REQUIRE(handles.size() == data.size());
        REQUIRE(handles.size() == set.size());

        for (std::size_t i = 0; i < handles.size(); ++i) {
            REQUIRE(set.Get(handles[i]) != nullptr);
            REQUIRE(set.Get(handles[i])->value == data[i].value);
        }
    }


    TEST_CASE("SparseSet remains synchronized after middle removal",
              "[SparseSet][Invariant][Lifecycle]") {
        osseus::SparseSet<TestData> set;

        const osseus::Handle first{10, 0};
        const osseus::Handle second{20, 0};
        const osseus::Handle third{30, 0};
        const osseus::Handle fourth{40, 0};

        set.Insert(first, TestData{100});
        set.Insert(second, TestData{200});
        set.Insert(third, TestData{300});
        set.Insert(fourth, TestData{400});

        set.Remove(second);

        const auto& handles = set.Handles();
        const auto& data = set.Data();

        REQUIRE(handles.size() == data.size());
        REQUIRE(handles.size() == set.size());

        for (std::size_t i = 0; i < handles.size(); ++i) {
            REQUIRE(set.Get(handles[i]) != nullptr);
            REQUIRE(set.Get(handles[i])->value == data[i].value);
        }

        REQUIRE_FALSE(set.Contains(second));
        REQUIRE(set.Get(first)->value == 100);
        REQUIRE(set.Get(third)->value == 300);
        REQUIRE(set.Get(fourth)->value == 400);
    }


    TEST_CASE("SparseSet maintains correct size through mutations",
              "[SparseSet][Invariant][Lifecycle]") {
        osseus::SparseSet<TestData> set;

        std::vector<osseus::Handle> handles;

        for (std::uint32_t i = 0; i < 100; ++i) {
            handles.push_back(osseus::Handle{i, 0});
            set.Insert(handles.back(), TestData{static_cast<int>(i)});
        }

        REQUIRE(set.size() == 100);

        for (std::size_t i = 0; i < handles.size(); i += 2) {
            set.Remove(handles[i]);
        }

        REQUIRE(set.size() == 50);

        for (std::size_t i = 0; i < handles.size(); ++i) {
            if (i % 2 == 0) {
                REQUIRE_FALSE(set.Contains(handles[i]));
            }
            else {
                REQUIRE(set.Contains(handles[i]));
                REQUIRE(set.Get(handles[i])->value == static_cast<int>(i));
            }
        }
    }


    TEST_CASE("SparseSet handles high index after many low indices",
              "[SparseSet][Stress]") {
        osseus::SparseSet<TestData> set;

        for (std::uint32_t i = 0; i < 100; ++i) {
            set.Insert(
                osseus::Handle{i, 0},
                TestData{static_cast<int>(i)}
            );
        }

        const osseus::Handle high{100000, 0};

        set.Insert(high, TestData{12345});

        REQUIRE(set.size() == 101);
        REQUIRE(set.Contains(high));
        REQUIRE(set.Get(high)->value == 12345);

        for (std::uint32_t i = 0; i < 100; ++i) {
            const osseus::Handle handle{i, 0};

            REQUIRE(set.Contains(handle));
            REQUIRE(set.Get(handle)->value == static_cast<int>(i));
        }
    }

}