#include <gtest/gtest.h>
#include <SlamCore/data/buffer_collection.h>


TEST(BufferCollection, collection_buffer_constructor) {

    size_t n = 100;
    std::vector<Eigen::Vector3d> points(n);
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> rgba(n);

    // Constructor from a single buffer
    slam::BufferCollection collection_0(slam::BufferWrapper::CreatePtr<Eigen::Vector3d>(
            points,
            slam::BuilderFromSingleElementData<Eigen::Vector3d>("xyz", {{"x", "y", "z"}}).Build()
    ));

    ASSERT_EQ(collection_0.element<Eigen::Vector3d>("xyz").size(), n);

    // Constructor from two buffers
    auto collection = slam::BufferCollection::Factory(
            slam::BufferWrapper::CreatePtr<Eigen::Vector3d>(
                    points,
                    slam::BuilderFromSingleElementData<Eigen::Vector3d>("xyz", {{"x", "y", "z"}}).Build()
            ),
            slam::BufferWrapper::CreatePtr<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>(
                    rgba,
                    slam::BuilderFromSingleElementData<Eigen::Vector4d>("rgba", {{"r", "g", "b", "a"}}).Build()
            )
    );

    ASSERT_EQ(collection.element<Eigen::Vector3d>("xyz").size(), n);
}


TEST(BufferCollection, merging_multiple_buffers) {

    // Generate the Data
    size_t n = 100;
    std::vector<std::array<unsigned char, 3>> rgb_data(n);
    for (auto &rgb: rgb_data) {
        for (auto i(0); i < 3; ++i)
            rgb[i] = static_cast<unsigned char>(rand() % 255);
    }

    auto xyz_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(n);
    for (auto &point: *xyz_ptr)
        point = Eigen::Vector3d::Random();

    // Build the Data views
    std::unique_ptr<slam::ItemBuffer>
            item_0_ptr = slam::BufferWrapper::CreatePtr(rgb_data,
                                                        slam::BuilderFromSingleElementData<std::array<unsigned char, 3>>(
                                                                "rgb", {{"r", "g", "b"}}).Build());

    std::unique_ptr<slam::ItemBuffer>
            item_1_ptr = slam::BufferWrapper::CreatePtr<Eigen::Vector3d>(xyz_ptr,
                                                                         slam::BuilderFromSingleElementData<Eigen::Vector3d>(
                                                                                 "xyz",
                                                                                 {{"x", "y", "z"}}).Build());
    auto collection = slam::BufferCollection::Factory(std::move(item_0_ptr),
                                                      std::move(item_1_ptr));
    auto collection_deep_copy = collection.DeepCopy();

    auto rgb_view = collection.element<std::array<unsigned char, 3>>("rgb");
    auto rgb_eigen_view = collection.element<Eigen::Matrix<unsigned char, 3, 1>>("rgb");
    auto rgb_eigen_view_copy = collection_deep_copy.element<Eigen::Matrix<unsigned char, 3, 1>>("rgb");
    auto xyz_view = collection.element<Eigen::Vector3d>("xyz");
    auto x_view = collection.property<double>("xyz", "x");
    auto y_view = collection.property<double>("xyz", "y");
    auto z_view = collection.property<double>("xyz", "z");


    ASSERT_EQ(rgb_data.size(), rgb_view.size());
    for (auto i(0); i < n; ++i) {
        for (auto k(0); k < 3; ++k) {
            ASSERT_EQ(rgb_view[i][k], rgb_data[i][k]);
            ASSERT_EQ(rgb_eigen_view_copy[i](k, 0), rgb_data[i][k]);
        }
    }

    for (auto i(0); i < n; ++i) {
        ASSERT_EQ(rgb_data[i][0], rgb_eigen_view[i].x());
        ASSERT_EQ(rgb_data[i][1], rgb_eigen_view[i].y());
        ASSERT_EQ(rgb_data[i][2], rgb_eigen_view[i].z());
    }

    for (auto i(0); i < n; ++i) {
        ASSERT_EQ(xyz_view[i].x(), xyz_ptr->at(i).x());
        ASSERT_EQ(xyz_view[i].y(), xyz_ptr->at(i).y());
        ASSERT_EQ(xyz_view[i].z(), xyz_ptr->at(i).z());
        ASSERT_EQ(x_view[i], xyz_ptr->at(i).x());
        ASSERT_EQ(y_view[i], xyz_ptr->at(i).y());
        ASSERT_EQ(z_view[i], xyz_ptr->at(i).z());
    }

    // Add a new buffer
    auto timestamps_buffer = std::make_unique<slam::VectorBuffer>(slam::ItemSchema::Builder(sizeof(double))
                                                                          .AddElement("other", 0)
                                                                          .AddScalarProperty<double>("other",
                                                                                                     "timestamp",
                                                                                                     0).Build(),
                                                                  sizeof(double));
    timestamps_buffer->Resize(xyz_view.size());
    collection.AddBuffer(std::move(timestamps_buffer));

    ASSERT_TRUE(collection.HasElement("other"));
    ASSERT_EQ(collection.NumItemsInSchema(), 3);
    ASSERT_TRUE(collection.GetItemInfo(2).HasElement("other"));
    ASSERT_FALSE(collection.IsResizable());
}


TEST(BufferCollection, resizable_buffers) {

    slam::ItemBufferPtr buffer_1 = std::make_unique<slam::VectorBuffer>(
            slam::ItemSchema::Builder(sizeof(Eigen::Vector3d))
                    .AddElement("xyz", 0)
                    .AddScalarProperty<double>("xyz", "x", 0)
                    .AddScalarProperty<double>("xyz", "y", sizeof(double))
                    .AddScalarProperty<double>("xyz", "z", 2 * sizeof(double))
                    .Build(),
            sizeof(Eigen::Vector3d));

    slam::ItemBufferPtr buffer_2 = std::make_unique<slam::VectorBuffer>(
            slam::ItemSchema::Builder(sizeof(std::array<char, 3>))
                    .AddElement("rgb", 0)
                    .AddScalarProperty<char>("rgb", "r", 0)
                    .AddScalarProperty<char>("rgb", "g", sizeof(char))
                    .AddScalarProperty<char>("rgb", "b", 2 * sizeof(char))
                    .Build(),
            sizeof(std::array<char, 3>));

    slam::BufferCollection collection = slam::BufferCollection::Factory(std::move(buffer_1), std::move(buffer_2));
    ASSERT_TRUE(collection.HasElement("xyz"));
    ASSERT_TRUE(collection.HasElement("rgb"));
    ASSERT_TRUE(collection.IsResizable());
    collection.Resize(100);
    ASSERT_EQ(collection.NumItemsPerBuffer(), 100);

    // Test the removing of an element
    collection.RemoveElement("rgb", true);
    ASSERT_EQ(collection.NumItemsInSchema(), 1);

    auto &collect = const_cast<const slam::BufferCollection &>(collection);
    const auto view = collect.item<Eigen::Vector3d>(0);
    ASSERT_EQ(view.size(), 100);

    auto cbegin = view.cbegin();
    auto cend = view.cend();

    auto result = std::accumulate(cbegin, cend, 0., [](double acc, const Eigen::Vector3d &mat) {
        return acc + mat.norm();
    });
    ASSERT_GE(result, 0.);
}