#include <gtest/gtest.h>
#include <SlamCore/data/schema.h>

struct CustomItem {

    // First Element Are properties
    // Note: Be careful of the order of the element in the struct to define properties
    //       The size of a struct might be different than the sum of the sizes of the element
    //       Further the elements of the struct, depending on their order might not be contiguous in memory
    long long int64_field;
    double double_field;
    int int_field;
    unsigned int uint_field;
    float float_field;
    short short_field;
    unsigned short ushort_field;
    char char_field;
    unsigned char uchar_field;


    //
    double xyz[3];
    int int10[10];
};


TEST(Schema, builder) {
    // -- Simple element with two basic properties
    slam::ItemSchema::Builder builder(sizeof(CustomItem));
    builder.AddElement("pair_int", offsetof(CustomItem, int_field));
    builder.AddScalarProperty<int>("pair_int", "int_field", 0);
    builder.AddScalarProperty<unsigned int>("pair_int", "uint_field", sizeof(CustomItem::int_field));
    builder.AddElement("pair_vector", offsetof(CustomItem, xyz));
    builder.AddProperty("pair_vector", "xyz", slam::StaticPropertyType<double>(), 0, 3);

    auto int10dim = sizeof(CustomItem::int10) / sizeof(decltype(CustomItem::int10[0]));
    builder.AddProperty("pair_vector", "int10", slam::StaticPropertyType<int>(),
                        sizeof(CustomItem::xyz), int10dim);

    ASSERT_NO_FATAL_FAILURE(builder.Build());

    auto schema = builder.Build();
    ASSERT_EQ(schema.GetItemSize(), sizeof(CustomItem));
    ASSERT_EQ(builder.GetItemSize(), sizeof(CustomItem));
    ASSERT_TRUE(builder.GetElement("pair_vector").HasProperty("xyz"));
    ASSERT_NO_THROW([&builder]() { builder.GetElement("pair_vector").CheckMemoryLayout(); }());

    ASSERT_EQ(schema.GetTotalElementSize(), sizeof(CustomItem::int_field) +
                                            sizeof(CustomItem::uint_field) +
                                            sizeof(CustomItem::int10) +
                                            sizeof(CustomItem::xyz));

    auto new_schema = schema.GetBuilder().RemoveElement("pair_vector").Build();
    ASSERT_EQ(new_schema.GetItemSize(), sizeof(CustomItem));
    ASSERT_EQ(new_schema.GetTotalElementSize(), sizeof(CustomItem::int_field) +
                                                sizeof(CustomItem::uint_field));

    ASSERT_TRUE(schema.HasElement("pair_vector"));
    ASSERT_FALSE(new_schema.HasElement("syz"));
    ASSERT_TRUE(schema.HasProperty("pair_vector", "xyz"));
    ASSERT_FALSE(new_schema.HasProperty("syz", "hli"));

    ASSERT_EQ(new_schema.GetElementNames().size(), 1);

    ASSERT_EQ(schema.GetPropertyInfo("pair_vector", "xyz").dimension, 3);
    ASSERT_EQ(schema.GetPropertyInfo("pair_vector", "xyz").type, slam::PROPERTY_TYPE::FLOAT64);
}


template<>
slam::ItemSchema::Builder slam::BuilderFromSingleElementData<CustomItem>(const std::string &element_name,
                                                                         std::optional<std::vector<std::string>> properties) {
    slam::ItemSchema::Builder builder;
    builder.SetItemSize(sizeof(CustomItem));
    builder.AddElement(element_name, 0);

    auto base_offset = offsetof(CustomItem, int64_field);
    builder.AddScalarProperty<int>(element_name, "int_field", offsetof(CustomItem, int_field) - base_offset);
    builder.AddScalarProperty<unsigned int>(element_name, "uint_field", offsetof(CustomItem, uint_field) - base_offset);
    builder.AddScalarProperty<double>(element_name, "double_field", offsetof(CustomItem, double_field) - base_offset);
    builder.AddScalarProperty<float>(element_name, "float_field", offsetof(CustomItem, float_field) - base_offset);
    builder.AddScalarProperty<long long>(element_name, "int64_field", offsetof(CustomItem, int64_field) - base_offset);
    builder.AddScalarProperty<short>(element_name, "short_field", offsetof(CustomItem, short_field) - base_offset);
    builder.AddScalarProperty<unsigned short>(element_name, "ushort_field",
                                              offsetof(CustomItem, ushort_field) - base_offset);
    builder.AddScalarProperty<char>(element_name, "char_field", offsetof(CustomItem, char_field) - base_offset);
    builder.AddScalarProperty<unsigned char>(element_name, "uchar_field",
                                             offsetof(CustomItem, uchar_field) - base_offset);

    return builder;
}

TEST(Schema, builder_from_data) {

    // Test the handcrafted overloading of slam::BuilderFromSingleElementData
    auto schema = slam::BuilderFromSingleElementData<CustomItem>("custom_item", {}).Build();
    ASSERT_EQ(schema.GetItemSize(), sizeof(CustomItem));
    std::cout << schema << std::endl;
    auto _schema = slam::BuilderFromSingleElementData<Eigen::Matrix3d>("matrix3d").Build();
    std::cout << _schema << std::endl;
    ASSERT_EQ(_schema.GetItemSize(), sizeof(Eigen::Matrix3d));
    ASSERT_EQ(_schema.GetElementInfo("matrix3d").GetProperty("matrix3d").dimension, 9);
    ASSERT_EQ(_schema.GetElementInfo("matrix3d").GetProperty("matrix3d").type, slam::PROPERTY_TYPE::FLOAT64);
}

