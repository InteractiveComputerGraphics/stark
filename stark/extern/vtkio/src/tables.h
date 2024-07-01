#pragma once
#include <array>
#include <string>
#include <limits>

#include "dynamic_types.h"

namespace vtkio
{
	enum class CellType
	{
		Empty = 0,
		Vertex = 1,
		PolyVertex = 2,
		Line = 3,
		PolyLine = 4,
		Triangle = 5,
		TriangleStrip = 6,
		Polygon = 7,
		Pixel = 8,
		Quad = 9,
		Tetra = 10,
		Voxel = 11,
		Hexahedron = 12,
		Wedge = 13,
		Pyramid = 14,
		PentaPrism = 15,
		HexaPrism = 16,
		Line3 = 21,
		Triangle6 = 22,
		Quad8 = 23,
		Tetra10 = 24,
		Hexahedron20 = 25,
		Wedge15 = 26,
		Pyramid13 = 27,
		Quad9 = 28,
		Hexahedron27 = 29,
		Quad6 = 30,
		Wedge12 = 31,
		Wedge18 = 32,
		Hexahedron24 = 33,
		Triangle7 = 34,
		Line4 = 35
	};
	enum class AttributeType
	{
		Scalars = 0,
		Vectors = 1,
		Tensors = 2,
		Normals = 3,
		TextureCoordinates = 4,
		FieldData = 5
	};

	namespace tables
	{
		// CELL TABLES =================================================================================
		struct CellTypeInfo
		{
			int vtk_type;
			bool supported;
			bool fixed_n_nodes;
			size_t n_nodes;
		};
		constexpr std::array<CellTypeInfo, 36> ELEMENT_TYPE_INFO =
		{{
			// vtk_type, supported, fixed_n_nodes, n_nodes

			{0,		false,	true,	0},  // empty
			{1,		true,	true,	1},  // vertex
			{2,		true,	false,	0},  // polyvertex
			{3,		true,	true,	2},  // line
			{4,		true,	false,	0},  // polyline
			{5,		true,	true,	3},  // triangle
			{6,		true,	false,	0},  // triangle strip
			{7,		true,	false,  0},  // polygon
			{8,		false,	true,	0},  // pixel
			{9,		true,	true,	4},  // quad
			{10,	true,	true,	4},  // tetra
			{11,	false,	true,	0},  // voxel
			{12,	true,	true,	8},  // hexahedron
			{13,	true,	true,	6},  // wedge
			{14,	true,	true,	5},  // pyramid
			{15,	true,	true,	10}, // penta_prism
			{16,	true,	true,	12}, // hexa_prism
			{17,	false,  false,	0},  // unknown
			{18,	false,  false,	0},  // unknown
			{19,	false,  false,	0},  // unknown
			{20,	false,  false,	0},  // unknown
			{21,	true,	true,	3},  // line3
			{22,	true,	true,	6},  // triangle6
			{23,	true,	true,	8},  // quad8
			{24,	true,	true,	10}, // tetra10
			{25,	true,	true,	20}, // hexahedron20
			{26,	true,	true,	15}, // wedge15
			{27,	true,	true,	13}, // pyramid13
			{28,	true,	true,	9},  // quad9
			{29,	true,	true,	27}, // hexahedron27
			{30,	true,	true,	6},  // quad6
			{31, 	true,	true,	12}, // wedge12
			{32, 	true,	true,	18}, // wedge18
			{33,	true,	true,	24}, // hexahedron24
			{34,	true,	true,	7},  // triangle7
			{35,	true,	true,	4}   // line4
		}};
		constexpr CellTypeInfo getVTKCellTypeInfo(const CellType cell_type)
		{
			return ELEMENT_TYPE_INFO[static_cast<int>(cell_type)];
		};
		constexpr CellTypeInfo getVTKCellTypeInfo(const int cell_type)
		{
			return ELEMENT_TYPE_INFO[cell_type];
		};


		// ATTRIBUTE TABLES =================================================================================
		struct AttributeTypeInfo
		{
			int attribute_type;
			bool supported;
			size_t min_dimensions;
			size_t max_dimensions;
		};
		constexpr size_t MAX_INT = std::numeric_limits<size_t>::max();
		constexpr std::array<AttributeTypeInfo, 6> ATTRIBUTE_TYPE_INFO =
		{ {
				// attribute_type, supported, max_dimensions

				{0,		true,	1,	4		},  // Scalars
				{1,		true,	3,	3		},  // Vectors
				{2,		true,	9,	9		},  // Tensors
				{3,		true,	3,	3		},  // Normals
				{4,		false,	1,	3		},  // TextureCoordinates
				{5,		true,	1,	MAX_INT	}   // FieldData
		} };
		constexpr AttributeTypeInfo getVTKAttributeTypeInfo(const AttributeType attr_type)
		{
			return ATTRIBUTE_TYPE_INFO[static_cast<const int>(attr_type)];
		};
		inline std::string getAttributeTypeString(const vtkio::AttributeType vtk_attribute_type)
		{
			switch (vtk_attribute_type) {
			case vtkio::AttributeType::Scalars: return "SCALARS";
			case vtkio::AttributeType::Vectors: return "VECTORS";
			case vtkio::AttributeType::Tensors: return "TENSORS";
			case vtkio::AttributeType::Normals: return "NORMALS";
			case vtkio::AttributeType::TextureCoordinates: return "TextureCoordinates";
			case vtkio::AttributeType::FieldData: return "FIELD";
			default:
				throw std::invalid_argument("Invalid VTK attribute type.");
				break;
			}
		};

		// DATATYPE TABLES =================================================================================
		inline std::string getVTKDataTypeString(const dynamic_types::DataType data_type)
		{
			switch (data_type) {
			case dynamic_types::DataType::Char: return "char";
			case dynamic_types::DataType::Int16: return "short";
			case dynamic_types::DataType::Int32: return "int";
			case dynamic_types::DataType::Int64: return "long";
			case dynamic_types::DataType::Uint16: return "unsigned_short";
			case dynamic_types::DataType::Uint32: return "unsigned_int";
			case dynamic_types::DataType::Uint64: return "unsigned_long";
			case dynamic_types::DataType::Float32: return "float";
			case dynamic_types::DataType::Float64: return "double";
			default:
				throw std::invalid_argument("VTK data type not supported.");
				break;
			}
		}
		inline dynamic_types::DataType getDataTypeFromString(const std::string type_str)
		{
			if      (type_str.substr(0, 4) == "char")  { return dynamic_types::DataType::Char; }
			else if (type_str.substr(0, 5) == "short") { return dynamic_types::DataType::Int16; }
			else if (type_str.substr(0, 3) == "int") { return dynamic_types::DataType::Int32; }
			else if (type_str.substr(0, 4) == "long") { return dynamic_types::DataType::Int64; }
			else if (type_str.substr(0, 14) == "unsigned_short") { return dynamic_types::DataType::Uint16; }
			else if (type_str.substr(0, 12) == "unsigned_int") { return dynamic_types::DataType::Uint32; }
			else if (type_str.substr(0, 13) == "unsigned_long") { return dynamic_types::DataType::Uint64; }
			else if (type_str.substr(0, 5) == "float") { return dynamic_types::DataType::Float32; }
			else if (type_str.substr(0, 6) == "double") { return dynamic_types::DataType::Float64; }
			else {
				throw std::invalid_argument("VTK data str type not supported.");
			}
		}
	}
};
