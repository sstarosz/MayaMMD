import unittest

from mmd.maya.pmx_naming_manager import PMXNamingManager


# Mock classes
class MockPmxHeader:
    def __init__(self, model_name_local="", model_name_universal=""):
        self.model_name_local = model_name_local
        self.model_name_universal = model_name_universal


class MockPmxMaterial:
    def __init__(self, name_local, name_universal):
        self.name_local = name_local
        self.name_universal = name_universal
        self.diffuse_color = None
        self.texture_index = 0
        self.face_vertex_count = 0


class MockPmxModel:
    def __init__(self, materials_data=None, textures_paths=None):
        self.header = MockPmxHeader()
        self.materials = []
        if materials_data:
            for mat_data in materials_data:
                material = MockPmxMaterial(
                    mat_data["name_local"], mat_data["name_universal"]
                )
                self.materials.append(material)
        self.textures_paths = textures_paths if textures_paths is not None else []
        self.absolute_path = ""
        self.morphs = []  # Ensure .morphs always exists
        self.rigid_bodies = []
        self.bones = []
        self.joints = []


class TestPMXNamingStrategy(unittest.TestCase):
    def test_get_texture_name(self):
        """Test get_texture_name returns correct sanitized and unique names."""

        # Test with various texture file names
        textures_paths = [
            "diffuse.png",  # Normal
            "diffuse 1.png",  # Space
            "diffuse+spec.png",  # Plus
            "テクスチャ.png",  # Non-ASCII
            "diffuse@.png",  # Special char
            "123abc.png",  # Starts with number
            "",  # Empty
        ]
        pmx_model = MockPmxModel(textures_paths=textures_paths)
        naming_manager = PMXNamingManager(pmx_model)

        expected_names = [
            "Model_diffuse_Tex",
            "Model_diffuse_1_Tex",
            "Model_diffuse_spec_Tex",
            "Model_Model_Tex",  # Non-ASCII sanitized to 'Model' (empty fallback), first empty
            "Model_diffuse_2_Tex",  # Third occurrence of sanitized 'diffuse'
            "Model_Model_123abc_Tex",  # Starts with digit -> 'Model_' prefix added
            "Model_Texture_Tex",  # Empty path -> 'Texture' fallback (first occurrence)
        ]
        for idx, expected in enumerate(expected_names):
            actual = naming_manager.get_texture_name(idx)
            self.assertEqual(
                actual, expected, f"Texture name mismatch at {idx}: {actual}"
            )

        # Out of range index
        self.assertEqual(naming_manager.get_texture_name(99), "Model_Texture_99_Tex")

    def test_get_place2d_name(self):
        """Test get_place2d_name returns correct names based on texture name."""

        textures_paths = [
            "diffuse.png",  # Normal
            "diffuse 1.png",  # Space
            "",  # Empty
        ]
        pmx_model = MockPmxModel(textures_paths=textures_paths)
        naming_manager = PMXNamingManager(pmx_model)

        # get_texture_name(0) -> PMX_diffuse_Tex, so place2d should be PMX_diffuse_Place2D
        self.assertEqual(
            naming_manager.get_place2d_name(0),
            "Model_diffuse_Place2D",
        )
        # get_texture_name(1) -> Model_diffuse_1_Tex, so place2d should be Model_diffuse_1_Place2D
        self.assertEqual(
            naming_manager.get_place2d_name(1),
            "Model_diffuse_1_Place2D",
        )
        # get_texture_name(2) -> Model_Texture_2_Tex, so place2d should be Model_Texture_2_Place2D
        self.assertEqual(
            naming_manager.get_place2d_name(2),
            "Model_Texture_Place2D",
        )
        # Out of range index
        self.assertEqual(
            naming_manager.get_place2d_name(99),
            "Model_Texture_99_Place2D",
        )

    def test_root_node_naming(self):
        """Test that the root node is named correctly."""
        pmx_model = MockPmxModel([])
        pmx_model.header.model_name_local = "TestModelLocal"
        pmx_model.header.model_name_universal = "TestModelUniversal"

        naming_manager = PMXNamingManager(pmx_model)

        expected_root_name = "TestModelLocal_Root"
        actual_root_name = naming_manager.get_root_name()

        self.assertEqual(
            actual_root_name,
            expected_root_name,
            f"Root node name mismatch. Expected: {expected_root_name}, Got: {actual_root_name}",
        )

    def test_geometry_node_naming(self):
        """Test that the geometry node is named correctly."""
        pmx_model = MockPmxModel([])
        pmx_model.header.model_name_local = "GeoModelLocal"
        pmx_model.header.model_name_universal = "GeoModelUniversal"

        naming_manager = PMXNamingManager(pmx_model)

        expected_geo_name = "GeoModelLocal_Geo"
        actual_geo_name = naming_manager.get_geo_group_name()

        self.assertEqual(
            actual_geo_name,
            expected_geo_name,
            f"Geometry node name mismatch. Expected: {expected_geo_name}, Got: {actual_geo_name}",
        )

        expected_mesh_name = "GeoModelLocal_Mesh"
        actual_mesh_name = naming_manager.get_mesh_name()
        self.assertEqual(
            actual_mesh_name,
            expected_mesh_name,
            f"Mesh name mismatch. Expected: {expected_mesh_name}, Got: {actual_mesh_name}",
        )

        expected_shape_name = "GeoModelLocal_Mesh_Shape"
        actual_shape_name = naming_manager.get_shape_name()
        self.assertEqual(
            actual_shape_name,
            expected_shape_name,
            f"Shape name mismatch. Expected: {expected_shape_name}, Got: {actual_shape_name}",
        )

    def test_material_naming_with_duplicates_sequential_numbering(self):
        """Test that duplicate material names are handled with sequential numbering."""
        materials_data = [
            {"name_local": "BodySkin", "name_universal": "BodySkin"},
            {"name_local": "Clothing", "name_universal": "Clothing"},
            {"name_local": "Clothing", "name_universal": "Clothing"},
            {"name_local": "Socks", "name_universal": "Socks"},
            {"name_local": "Lashes", "name_universal": "Lashes"},
            {"name_local": "Brows", "name_universal": "Brows"},
            {"name_local": "Teeth", "name_universal": "Teeth"},
            {"name_local": "Teeth", "name_universal": "Teeth"},
            {"name_local": "Tongue", "name_universal": "Tongue"},
            {"name_local": "Mouth", "name_universal": "Mouth"},
            {"name_local": "Face", "name_universal": "Face"},
            {"name_local": "EyeWhite", "name_universal": "EyeWhite"},
            {"name_local": "Eyes", "name_universal": "Eyes"},
            {"name_local": "Eyes+", "name_universal": "Eyes+"},
            {"name_local": "EyeShadow", "name_universal": "EyeShadow"},
            {"name_local": "Hair", "name_universal": "Hair"},
            {"name_local": "Hair", "name_universal": "Hair"},
            {"name_local": "Clothing", "name_universal": "Clothing"},
            {"name_local": "Clothing", "name_universal": "Clothing"},
            {"name_local": "Emotion1", "name_universal": "Emotion1"},
            {"name_local": "Emotion2", "name_universal": "Emotion2"},
        ]

        pmx_model = MockPmxModel(materials_data)
        naming_manager = PMXNamingManager(pmx_model)

        # Expected material names with sequential numbering for duplicates
        expected_material_names = [
            "Model_BodySkin_Mat",  # 0: Unique
            "Model_Clothing_Mat",  # 1: First "Clothing"
            "Model_Clothing_1_Mat",  # 2: Second "Clothing" - gets _1
            "Model_Socks_Mat",  # 3: Unique
            "Model_Lashes_Mat",  # 4: Unique
            "Model_Brows_Mat",  # 5: Unique
            "Model_Teeth_Mat",  # 6: First "Teeth"
            "Model_Teeth_1_Mat",  # 7: Second "Teeth" - gets _1
            "Model_Tongue_Mat",  # 8: Unique
            "Model_Mouth_Mat",  # 9: Unique
            "Model_Face_Mat",  # 10: Unique
            "Model_EyeWhite_Mat",  # 11: Unique
            "Model_Eyes_Mat",  # 12: Unique
            "Model_Eyes_1_Mat",  # 13: "Eyes+" -> "Eyes_" (plus becomes underscore)
            "Model_EyeShadow_Mat",  # 14: Unique
            "Model_Hair_Mat",  # 15: First "Hair"
            "Model_Hair_1_Mat",  # 16: Second "Hair" - gets _1
            "Model_Clothing_2_Mat",  # 17: Third "Clothing" - gets _2
            "Model_Clothing_3_Mat",  # 18: Fourth "Clothing" - gets _3
            "Model_Emotion1_Mat",  # 19: Unique
            "Model_Emotion2_Mat",  # 20: Unique
        ]

        # Test each material name
        for idx, expected_name in enumerate(expected_material_names):
            actual_name = naming_manager.get_material_name(idx)
            self.assertEqual(
                actual_name,
                expected_name,
                f"Material {idx} name mismatch. Expected: {expected_name}, Got: {actual_name}",
            )

        # Test shading group names
        expected_shading_group_names = [
            "Model_BodySkin_SG",  # 0
            "Model_Clothing_SG",  # 1
            "Model_Clothing_1_SG",  # 2
            "Model_Socks_SG",  # 3
            "Model_Lashes_SG",  # 4
            "Model_Brows_SG",  # 5
            "Model_Teeth_SG",  # 6
            "Model_Teeth_1_SG",  # 7
            "Model_Tongue_SG",  # 8
            "Model_Mouth_SG",  # 9
            "Model_Face_SG",  # 10
            "Model_EyeWhite_SG",  # 11
            "Model_Eyes_SG",  # 12
            "Model_Eyes_1_SG",  # 13
            "Model_EyeShadow_SG",  # 14
            "Model_Hair_SG",  # 15
            "Model_Hair_1_SG",  # 16
            "Model_Clothing_2_SG",  # 17
            "Model_Clothing_3_SG",  # 18
            "Model_Emotion1_SG",  # 19
            "Model_Emotion2_SG",  # 20
        ]

        for idx, expected_sg_name in enumerate(expected_shading_group_names):
            actual_sg_name = naming_manager.get_shading_group_name(idx)
            self.assertEqual(
                actual_sg_name,
                expected_sg_name,
                f"Shading group {idx} name mismatch. Expected: {expected_sg_name}, Got: {actual_sg_name}",
            )

    def test_material_naming_special_characters(self):
        """Test material names with special characters."""
        materials_data = [
            {"name_local": "Mat/Test", "name_universal": "MatTest"},  # Forward slash
            {"name_local": "Mat*Test", "name_universal": "MatTest"},  # Asterisk
            {"name_local": "Mat:Test", "name_universal": "MatTest"},  # Colon
            {"name_local": "Mat Test", "name_universal": "MatTest"},  # Space
            {
                "name_local": "Mat_Test",
                "name_universal": "MatTest",
            },  # Underscore (valid)
        ]

        pmx_model = MockPmxModel(materials_data)
        naming_manager = PMXNamingManager(pmx_model)

        # All should sanitize to "Mat_Test_Mat"
        expected_names = [
            "Model_Mat_Test_Mat",  # Forward slash -> underscore
            "Model_Mat_Test_1_Mat",  # Asterisk -> underscore
            "Model_Mat_Test_2_Mat",  # Colon -> underscore
            "Model_Mat_Test_3_Mat",  # Space -> underscore
            "Model_Mat_Test_4_Mat",  # Underscore preserved
        ]

        for idx, expected_name in enumerate(expected_names):
            actual_name = naming_manager.get_material_name(idx)
            self.assertEqual(actual_name, expected_name)

    def test_duplicate_sanitized_names(self):
        """Test when different original names sanitize to the same name."""
        materials_data = [
            {"name_local": "Material-1", "name_universal": "Material1"},
            {"name_local": "Material_1", "name_universal": "Material1"},
            {"name_local": "Material 1", "name_universal": "Material1"},
        ]

        pmx_model = MockPmxModel(materials_data)
        naming_manager = PMXNamingManager(pmx_model)

        # All sanitize to "Material_1", but should be sequentially numbered
        names = [naming_manager.get_material_name(i) for i in range(3)]

        self.assertEqual(names[0], "Model_Material_1_Mat")  # First occurrence
        self.assertEqual(names[1], "Model_Material_1_1_Mat")  # Second occurrence
        self.assertEqual(names[2], "Model_Material_1_2_Mat")  # Third occurrence

        # All should be unique
        self.assertEqual(len(set(names)), 3)

    def test_empty_and_none_names(self):
        """Test materials with empty or None names."""
        materials_data = [
            {"name_local": "", "name_universal": ""},
            {"name_local": None, "name_universal": None},
            {"name_local": "Valid", "name_universal": "Valid"},
        ]

        # Fix the data to use actual None values
        materials_data = [
            {"name_local": "", "name_universal": ""},
            {"name_local": None, "name_universal": None},
            {"name_local": "Valid", "name_universal": "Valid"},
        ]

        pmx_model = MockPmxModel(materials_data)
        # Set actual None values
        pmx_model.materials[1].name_local = None
        pmx_model.materials[1].name_universal = None

        naming_manager = PMXNamingManager(pmx_model)

        # Should fall back to Material_0, Material_1, etc.
        self.assertEqual(naming_manager.get_material_name(0), "Model_Material_0_Mat")
        self.assertEqual(naming_manager.get_material_name(1), "Model_Material_1_Mat")
        self.assertEqual(naming_manager.get_material_name(2), "Model_Valid_Mat")

    def test_rigid_body_node_naming(self):
        """Test rigid bodies group + solver names (local / universal / fallback)."""
        # Local name wins when present.
        pmx_model = MockPmxModel([])
        pmx_model.header.model_name_local = "PhysModelLocal"
        pmx_model.header.model_name_universal = "PhysModelUniversal"
        naming_manager = PMXNamingManager(pmx_model)

        self.assertEqual(
            naming_manager.get_rigid_bodies_group_name(), "PhysModelLocal_RigidBodies"
        )
        self.assertEqual(
            naming_manager.get_rigid_body_solver_name(),
            "PhysModelLocal_RigidBodySolver",
        )

        # Universal name is the fallback when local is empty.
        pmx_model2 = MockPmxModel([])
        pmx_model2.header.model_name_local = ""
        pmx_model2.header.model_name_universal = "PhysModelUniversal"
        naming_manager2 = PMXNamingManager(pmx_model2)

        self.assertEqual(
            naming_manager2.get_rigid_bodies_group_name(),
            "PhysModelUniversal_RigidBodies",
        )
        self.assertEqual(
            naming_manager2.get_rigid_body_solver_name(),
            "PhysModelUniversal_RigidBodySolver",
        )

        # No model names at all -> bare fallback names.
        naming_manager3 = PMXNamingManager(MockPmxModel([]))
        self.assertEqual(naming_manager3.get_rigid_bodies_group_name(), "RigidBodies")
        self.assertEqual(
            naming_manager3.get_rigid_body_solver_name(), "RigidBodySolver"
        )


if __name__ == "__main__":
    unittest.main()
