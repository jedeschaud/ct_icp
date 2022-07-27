#include <gtest/gtest.h>

#include <SlamCore/data/buffer_collection.h>
#include "SlamCore/types.h"

struct CustomItem {
    int int_field;
    uint8_t uint_field;
    double double_field;
    char char_field;
    unsigned char uchar_field;

    // Arrays
    double xyz[3];
    int int10[10];
};

/* ----------------------------------------------------------------------------------------------------------------- */
std::vector<CustomItem> RandomData(int num_items) {
    // Generate random data
    std::vector<CustomItem> random_data(num_items);
    unsigned char *data = reinterpret_cast<unsigned char *>(&random_data[0]);
    const int kDTypeSize = sizeof(CustomItem);
    for (auto i(0); i < random_data.size(); ++i) {
        auto item_ptr = data + kDTypeSize * i;
        for (int k(0); k < kDTypeSize; ++k) {
            item_ptr[k] = static_cast<unsigned char>(rand() % 255);
        }
    }
    return random_data;
}



/* ------------------------------------------------------------------------------------------------------------------ */
// Test the VectorBuffer for managing custom items
TEST(DataLayout, VectorBuffer) {

    auto random_data = RandomData(10);

    slam::ItemSchema::Builder builder(sizeof(CustomItem));
    builder.AddElement("pair_int", offsetof(CustomItem, int_field));
    builder.AddScalarProperty<int>("pair_int", "int_field", 0);
    builder.AddScalarProperty<unsigned int>("pair_int", "uint_field", sizeof(CustomItem::int_field));

    builder.AddElement("pair_vector", offsetof(CustomItem, xyz));
    builder.AddProperty("pair_vector", "xyz", slam::StaticPropertyType<double>(), 0, 3);
    auto int10dim = sizeof(CustomItem::int10) / sizeof(int);
    builder.AddProperty("pair_vector", "int10", slam::StaticPropertyType<int>(),
                        sizeof(CustomItem::xyz), int10dim);
    // Build the vector
    slam::VectorBuffer buffer(builder.Build(), sizeof(CustomItem));
    // buffer.Reserve(random_data.size());
    for (auto i(0); i < random_data.size(); ++i) {
        buffer.PushBackItem(random_data[i]);
    }

    // Verify that it contains the same elements
    ASSERT_EQ(random_data.size(), buffer.NumItems());
    auto check_data = [&] {
        for (auto i(0); i < random_data.size(); ++i) {

            char *item = reinterpret_cast<char *>(&random_data[i]);
            char *item_copy = reinterpret_cast<char *>(&buffer.At<CustomItem>(i));
            for (auto k(0); k < sizeof(CustomItem); ++k)
                ASSERT_EQ(item[k], item_copy[k]);
        }
    };
    check_data();

    ASSERT_TRUE(buffer.IsResizable());
    buffer.Reserve(15);
    buffer.Resize(15);
    check_data();

    for (auto i(10); i < 15; ++i) {
        char *item_copy = reinterpret_cast<char *>(&buffer.At<CustomItem>(i));
        for (auto k(0); k < sizeof(CustomItem); ++k)
            ASSERT_EQ(static_cast<char>(0), item_copy[k]);
    }

}

void f(slam::View<int> &);

/* ------------------------------------------------------------------------------------------------------------------ */
// Test the  for managing custom items
TEST(DataLayout, BufferWrapper) {
    auto random_data = RandomData(10);

    slam::ItemSchema::Builder builder(sizeof(CustomItem));
    builder.AddElement("pair_int", offsetof(CustomItem, int_field));
    builder.AddScalarProperty<int>("pair_int", "int_field", 0);
    builder.AddScalarProperty<unsigned int>("pair_int", "uint_field", sizeof(CustomItem::int_field));

    builder.AddElement("pair_vector", offsetof(CustomItem, xyz));
    builder.AddProperty("pair_vector", "xyz", slam::StaticPropertyType<double>(), 0, 3);
    auto int10dim = sizeof(CustomItem::int10) / sizeof(int);
    builder.AddProperty("pair_vector", "int10", slam::StaticPropertyType<int>(),
                        sizeof(CustomItem::xyz), int10dim);

    // Build the vector
    auto buffer_wrapper = slam::BufferWrapper::Create(random_data, builder.Build());

    // Verify that it contains the same elements
    ASSERT_EQ(random_data.size(), buffer_wrapper.NumItems());
    for (auto i(0); i < random_data.size(); ++i) {

        char *item = reinterpret_cast<char *>(&random_data[i]);
        char *item_copy = reinterpret_cast<char *>(&buffer_wrapper.At<CustomItem>(i));
        for (auto k(0); k < sizeof(CustomItem); ++k)
            ASSERT_EQ(item[k], item_copy[k]);
    }

    auto &ref_xyz = buffer_wrapper.At<CustomItem>(0).xyz;
    ref_xyz[0] = 42.;
    ref_xyz[1] = 67.;
    ref_xyz[2] = 75.;
    ASSERT_EQ(ref_xyz[0], random_data[0].xyz[0]);
    ASSERT_EQ(ref_xyz[1], random_data[0].xyz[1]);
    ASSERT_EQ(ref_xyz[2], random_data[0].xyz[2]);
}


/* ------------------------------------------------------------------------------------------------------------------ */
// Test the  for managing custom items
TEST(BufferWrapper, WPoint3D) {
    std::vector<slam::WPoint3D> points(100);
    for (auto &point: points) {
        point.raw_point.point = Eigen::Vector3d::Random();
        point.world_point = Eigen::Vector3d::Random();
    }

    auto buffer = slam::VectorBuffer::Copy(points, slam::WPoint3D::DefaultSchema());
    auto buffer_copy = slam::VectorBuffer::DeepCopy(buffer);
    auto buffer_copy_ptr = slam::VectorBuffer::DeepCopyPtr(buffer);

    ASSERT_EQ(buffer.NumItems(), buffer_copy.NumItems());
    ASSERT_EQ(buffer.NumItems(), buffer_copy_ptr->NumItems());

    auto view = slam::View<slam::WPoint3D>(buffer, 0, sizeof(slam::WPoint3D));
    auto view_copy = slam::View<slam::WPoint3D>(buffer_copy, 0, sizeof(slam::WPoint3D));
    auto view_copy_ptr = slam::View<slam::WPoint3D>(*buffer_copy_ptr, 0, sizeof(slam::WPoint3D));

    for (auto i(0); i < view.size(); ++i) {
        ASSERT_EQ((view[i].world_point - points[i].world_point).norm(), 0.);
        ASSERT_EQ((view[i].RawPoint() - points[i].RawPoint()).norm(), 0.);
        ASSERT_EQ((view_copy[i].RawPoint() - points[i].RawPoint()).norm(), 0.);
        ASSERT_EQ((view_copy_ptr[i].RawPoint() - points[i].RawPoint()).norm(), 0.);
    }

}


/* ------------------------------------------------------------------------------------------------------------------ */
// Test the  for managing custom items
TEST(ArrayBuffer, WPoint3D) {
    slam::ArrayBuffer<10000> buffer(slam::WPoint3D::DefaultSchema(), sizeof(slam::WPoint3D));
    size_t capacity = buffer.MaxCapacity();

    std::vector<slam::WPoint3D> points;
    for (auto i(0); i < capacity; ++i) {
        slam::WPoint3D new_point;
        new_point.world_point = Eigen::Vector3d::Random();
        new_point.RawPoint() = Eigen::Vector3d::Random();
        points.push_back(new_point);
        buffer.PushBackItem(new_point);
    }
    ASSERT_GE(capacity, 0);
    for (auto i(0); i < capacity; ++i) {
        ASSERT_EQ((points[i].world_point - buffer.At<slam::WPoint3D>(i).world_point).norm(), 0.);
        ASSERT_EQ((points[i].RawPoint() - buffer.At<slam::WPoint3D>(i).RawPoint()).norm(), 0.);
    }

}