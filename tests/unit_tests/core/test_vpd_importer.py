# pylint: disable=missing-function-docstring,missing-module-docstring

import pytest

from mmd.core.vpd_importer import (
    VPDBoneParseError,
    VPDHeaderParseError,
    VPDParseError,
    _clean_data,
    _parse_bones,
    _parse_header,
    parse_vpd_file,
)

# --- Clean Data Tests ---


def test_clean_data_removes_comments():
    """Test that comments are properly removed from VPD data"""
    data = """Vocaloid Pose Data file
Model.pmx;    // This is a comment
5;            // Another comment
Bone0{TestBone  // inline comment
  1.0,2.0,3.0;  // position comment
  0.0,0.0,0.0,1.0;  // rotation comment
}"""
    cleaned = _clean_data(data)
    _lines = cleaned.split("\n")

    # Verify comments are removed
    assert "//" not in cleaned
    assert "This is a comment" not in cleaned
    assert "inline comment" not in cleaned


def test_clean_data_removes_empty_lines():
    """Test that empty lines are removed"""
    data = """Vocaloid Pose Data file

Model.pmx;

5;

Bone0{TestBone
  1.0,2.0,3.0;
  0.0,0.0,0.0,1.0;
}"""
    cleaned = _clean_data(data)
    lines = cleaned.split("\n")

    # No empty lines should remain
    assert all(line.strip() for line in lines)


def test_clean_data_strips_whitespace():
    """Test that leading/trailing whitespace is stripped"""
    data = """  Vocaloid Pose Data file  
    Model.pmx;    
    5;    
Bone0{TestBone
  1.0,2.0,3.0;
  0.0,0.0,0.0,1.0;
}"""
    cleaned = _clean_data(data)
    lines = cleaned.split("\n")

    # First line should be clean
    assert lines[0] == "Vocaloid Pose Data file"
    assert lines[1] == "Model.pmx;"


def test_clean_data_preserves_japanese_characters():
    """Test that Japanese characters in bone names are preserved"""
    data = """Vocaloid Pose Data file
モデル.pmx;
2;
Bone0{センター
  0.0,0.0,0.0;
  0.0,0.0,0.0,1.0;
}
Bone1{右腕
  1.0,2.0,3.0;
  0.0,0.0,0.0,1.0;
}"""
    cleaned = _clean_data(data)

    # Japanese characters should be preserved
    assert "センター" in cleaned
    assert "右腕" in cleaned
    assert "モデル" in cleaned


# --- Parse Header Tests ---


def test_parse_header_valid():
    """Test parsing a valid VPD header"""
    data = """Vocaloid Pose Data file
YYB式初音ミク_10th_v1.02.osm;
369;"""
    header = _parse_header(data)

    assert header["model_name"] == "YYB式初音ミク_10th_v1.02.osm;"
    assert header["bone_count"] == 369


def test_parse_header_bone_count_without_semicolon():
    """Test parsing header when bone count doesn't have semicolon"""
    data = """Vocaloid Pose Data file
Model.pmx;
10"""
    header = _parse_header(data)

    assert header["bone_count"] == 10


def test_parse_header_bone_count_with_semicolon():
    """Test parsing header when bone count has semicolon"""
    data = """Vocaloid Pose Data file
Model.pmx;
10;"""
    header = _parse_header(data)

    assert header["bone_count"] == 10


def test_parse_header_invalid_magic():
    """Test that invalid header magic raises error"""
    data = """Invalid Header
Model.pmx;
10;"""
    with pytest.raises(
        VPDHeaderParseError, match="does not start with expected header line"
    ):
        _parse_header(data)


def test_parse_header_too_short():
    """Test that file with too few lines raises error"""
    data = """Vocaloid Pose Data file
Model.pmx;"""
    with pytest.raises(VPDHeaderParseError, match="too short to contain valid header"):
        _parse_header(data)


def test_parse_header_invalid_bone_count():
    """Test that non-numeric bone count raises error"""
    data = """Vocaloid Pose Data file
Model.pmx;
not_a_number;"""
    with pytest.raises(VPDHeaderParseError, match="not a valid integer"):
        _parse_header(data)


def test_parse_header_empty_model_name():
    """Test parsing header with empty model name"""
    data = """Vocaloid Pose Data file
;
5;"""
    header = _parse_header(data)

    assert header["model_name"] == ";"
    assert header["bone_count"] == 5


# --- Parse Bones Tests ---


def test_parse_bones_single_bone():
    """Test parsing a single bone entry"""
    data = """Bone0{センター
  0.000000,0.000000,0.000000;
  0.000000,0.000000,0.000000,1.000000;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.bone_idx == "Bone0"
    assert bone.bone_name == "センター"
    assert bone.position.x == pytest.approx(0.0)
    assert bone.position.y == pytest.approx(0.0)
    assert bone.position.z == pytest.approx(0.0)
    assert bone.rotation.x == pytest.approx(0.0)
    assert bone.rotation.y == pytest.approx(0.0)
    assert bone.rotation.z == pytest.approx(0.0)
    assert bone.rotation.w == pytest.approx(1.0)


def test_parse_bones_multiple_bones():
    """Test parsing multiple bone entries"""
    data = """Bone0{操作中心
  10.000004,0.000000,10.549998;
  0.000000,0.000000,0.000000,1.000000;
}

Bone1{全ての親
  9.949996,0.000000,10.350008;
  0.000000,-0.000000,-0.000000,1.000000;
}

Bone2{センター
  0.000000,-1.300000,0.000000;
  0.000000,0.000000,0.000000,1.000000;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 3

    # Check first bone
    assert bones[0].bone_idx == "Bone0"
    assert bones[0].bone_name == "操作中心"
    assert bones[0].position.x == pytest.approx(10.000004)

    # Check second bone
    assert bones[1].bone_idx == "Bone1"
    assert bones[1].bone_name == "全ての親"
    assert bones[1].position.x == pytest.approx(9.949996)

    # Check third bone
    assert bones[2].bone_idx == "Bone2"
    assert bones[2].bone_name == "センター"
    assert bones[2].position.y == pytest.approx(-1.3)


def test_parse_bones_negative_values():
    """Test parsing bones with negative position and rotation values"""
    data = """Bone0{TestBone
  -1.5,-2.3,-3.7;
  -0.191924,-0.241775,0.090279,0.946868;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.position.x == pytest.approx(-1.5)
    assert bone.position.y == pytest.approx(-2.3)
    assert bone.position.z == pytest.approx(-3.7)
    assert bone.rotation.x == pytest.approx(-0.191924)
    assert bone.rotation.y == pytest.approx(-0.241775)
    assert bone.rotation.z == pytest.approx(0.090279)
    assert bone.rotation.w == pytest.approx(0.946868)


def test_parse_bones_non_zero_rotation():
    """Test parsing bones with non-identity quaternion rotation"""
    data = """Bone11{上半身
  0.000000,0.000000,0.000000;
  0.191924,0.241775,-0.090279,0.946868;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.bone_idx == "Bone11"
    assert bone.bone_name == "上半身"
    assert bone.rotation.x == pytest.approx(0.191924)
    assert bone.rotation.y == pytest.approx(0.241775)
    assert bone.rotation.z == pytest.approx(-0.090279)
    assert bone.rotation.w == pytest.approx(0.946868)


def test_parse_bones_large_index():
    """Test parsing bones with large index numbers"""
    data = """Bone368{前髪9
  0.000000,0.000000,0.000000;
  0.000000,0.000000,0.000000,1.000000;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.bone_idx == "Bone368"
    assert bone.bone_name == "前髪9"


def test_parse_bones_with_spaces_in_values():
    """Test parsing bones with extra spaces between values"""
    data = """Bone0{TestBone
  1.0  ,  2.0  ,  3.0  ;
  0.0  ,  0.0  ,  0.0  ,  1.0  ;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.position.x == pytest.approx(1.0)
    assert bone.position.y == pytest.approx(2.0)
    assert bone.position.z == pytest.approx(3.0)


def test_parse_bones_invalid_translation_count():
    """Test that invalid number of translation values raises error"""
    data = """Bone0{TestBone
  1.0,2.0;
  0.0,0.0,0.0,1.0;
}"""
    with pytest.raises(VPDBoneParseError, match="Invalid translation values"):
        _parse_bones(data)


def test_parse_bones_invalid_rotation_count():
    """Test that invalid number of rotation values raises error"""
    data = """Bone0{TestBone
  1.0,2.0,3.0;
  0.0,0.0,1.0;
}"""
    with pytest.raises(VPDBoneParseError, match="Invalid rotation values"):
        _parse_bones(data)


def test_parse_bones_empty_data():
    """Test parsing with no bone entries"""
    data = """Vocaloid Pose Data file
Model.pmx;
0;"""
    bones = _parse_bones(data)

    assert len(bones) == 0


def test_parse_bones_english_names():
    """Test parsing bones with English names"""
    data = """Bone0{Center
  0.0,0.0,0.0;
  0.0,0.0,0.0,1.0;
}
Bone1{LeftArm
  1.0,2.0,3.0;
  0.1,0.2,0.3,0.9;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 2
    assert bones[0].bone_name == "Center"
    assert bones[1].bone_name == "LeftArm"


def test_parse_bones_mixed_language_names():
    """Test parsing bones with mixed Japanese and English names"""
    data = """Bone0{Center_センター
  0.0,0.0,0.0;
  0.0,0.0,0.0,1.0;
}
Bone1{右Arm_Right
  1.0,2.0,3.0;
  0.0,0.0,0.0,1.0;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 2
    assert bones[0].bone_name == "Center_センター"
    assert bones[1].bone_name == "右Arm_Right"


def test_parse_bones_special_characters():
    """Test parsing bones with special characters in names"""
    data = """Bone0{Bone_123
  0.0,0.0,0.0;
  0.0,0.0,0.0,1.0;
}
Bone1{Bone-Test
  1.0,2.0,3.0;
  0.0,0.0,0.0,1.0;
}
Bone2{Bone.Test
  1.0,2.0,3.0;
  0.0,0.0,0.0,1.0;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 3
    assert bones[0].bone_name == "Bone_123"
    assert bones[1].bone_name == "Bone-Test"
    assert bones[2].bone_name == "Bone.Test"


def test_parse_bones_zero_values():
    """Test parsing bones with all zero values"""
    data = """Bone0{NullBone
  0.0,0.0,0.0;
  0.0,0.0,0.0,0.0;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.position.x == pytest.approx(0.0)
    assert bone.position.y == pytest.approx(0.0)
    assert bone.position.z == pytest.approx(0.0)
    assert bone.rotation.x == pytest.approx(0.0)
    assert bone.rotation.y == pytest.approx(0.0)
    assert bone.rotation.z == pytest.approx(0.0)
    assert bone.rotation.w == pytest.approx(0.0)


def test_parse_bones_very_small_values():
    """Test parsing bones with very small floating point values"""
    data = """Bone0{TestBone
  0.000001,0.000002,0.000003;
  0.000001,0.000002,0.000003,0.999999;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.position.x == pytest.approx(0.000001)
    assert bone.position.y == pytest.approx(0.000002)
    assert bone.position.z == pytest.approx(0.000003)
    assert bone.rotation.x == pytest.approx(0.000001)
    assert bone.rotation.w == pytest.approx(0.999999)


def test_parse_bones_large_values():
    """Test parsing bones with large values"""
    data = """Bone0{TestBone
  100.5,200.7,300.9;
  0.5,0.5,0.5,0.5;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.position.x == pytest.approx(100.5)
    assert bone.position.y == pytest.approx(200.7)
    assert bone.position.z == pytest.approx(300.9)


def test_parse_bones_preserves_order():
    """Test that bone order is preserved during parsing"""
    data = """Bone5{Fifth
  5.0,5.0,5.0;
  0.0,0.0,0.0,1.0;
}
Bone2{Second
  2.0,2.0,2.0;
  0.0,0.0,0.0,1.0;
}
Bone9{Ninth
  9.0,9.0,9.0;
  0.0,0.0,0.0,1.0;
}"""
    bones = _parse_bones(data)

    assert len(bones) == 3
    # Order should match file order, not bone index
    assert bones[0].bone_idx == "Bone5"
    assert bones[1].bone_idx == "Bone2"
    assert bones[2].bone_idx == "Bone9"


def test_parse_vpd_file_not_found():
    with pytest.raises(VPDParseError, match="Failed to parse VPD file"):
        parse_vpd_file("nonexistent_file.vpd")


def test_parse_vpd_file_invalid_content(tmp_path):
    invalid_vpd = tmp_path / "invalid_content.vpd"
    invalid_vpd.write_text(
        "Invalid Header\nModel.pmx;\n10;\n",
        encoding="utf-8",
    )

    with pytest.raises(VPDParseError, match="Failed to parse VPD file"):
        parse_vpd_file(str(invalid_vpd))
