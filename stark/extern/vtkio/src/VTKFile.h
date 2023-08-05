#pragma once
#include <vector>
#include <unordered_map>
#include <string>
#include <type_traits>
#include <stdexcept>
#include <iostream>

#include "endianness.h"
#include "ByteBuffer.h"
#include "tables.h"

namespace vtkio
{
	using namespace dynamic_types;
	using namespace endianness;

	// Declarations =====================================================================================
	struct VTKData
	{
		vtkio::AttributeType attr_type;
		bytebuffer::ByteBuffer buffer;
	};

	class VTKFile
	{
		/* Fields */
	public:
		// These data is stored as it is writen/read in the VTK file format
		// The user can directly modify these buffers to operate VTKFile without using the provided interface methods
		bytebuffer::ByteBuffer points;
		bytebuffer::ByteBuffer cells;
		bytebuffer::ByteBuffer cell_types;
		std::unordered_map<std::string, VTKData> point_data;
		std::unordered_map<std::string, VTKData> cell_data;
		std::string comments;

		/* Methods */
	public:
		VTKFile() {};
		~VTKFile() {};

		// Setters -----------------------------------------------------------------------
		template<class T>
		void set_points_raw(const std::vector<T>& points_3d_flat);
		template<class TwiceIndexable>
		void set_points_from_twice_indexable(const TwiceIndexable& twice_indexable);
		template<class TwiceIndexable>
		void set_points_from_twice_indexable(const TwiceIndexable& twice_indexable, const size_t begin, const size_t end);

		void set_cells_raw(const std::vector<int>& cell_array_flat);
		void set_cell_types_raw(const std::vector<int>& cell_type_array_flat);
		template<class TwiceIndexable>
		void set_cells_from_twice_indexable(const TwiceIndexable& twice_indexable, const vtkio::CellType cell_type);
		template<class TwiceIndexable>
		void set_cells_from_twice_indexable(const TwiceIndexable& twice_indexable, const size_t begin, const size_t end, const vtkio::CellType cell_type);
		// For convenience, use this method when the cell type is CellType::Vertex
		void set_cells_as_particles(const size_t n_particles);

		template<class T>
		void set_point_data_raw(const std::string label, const std::vector<T>& data_flat, const vtkio::AttributeType attr_type);
		template<class Indexable>
		void set_point_data_from_indexable(const std::string label, const Indexable& indexable, const vtkio::AttributeType attr_type);
		template<class Indexable>
		void set_point_data_from_indexable(const std::string label, const Indexable& indexable, const size_t begin, const size_t end, const vtkio::AttributeType attr_type);
		template<class TwiceIndexable>
		void set_point_data_from_twice_indexable(const std::string label, const TwiceIndexable& twice_indexable, const vtkio::AttributeType attr_type);
		template<class TwiceIndexable>
		void set_point_data_from_twice_indexable(const std::string label, const TwiceIndexable& twice_indexable, const size_t begin, const size_t end, const vtkio::AttributeType attr_type);

		template<class T>
		void set_cell_data_raw(const std::string label, const std::vector<T>& data_flat, const vtkio::AttributeType attr_type);
		template<class Indexable>
		void set_cell_data_from_indexable(const std::string label, const Indexable& indexable, const vtkio::AttributeType attr_type);
		template<class Indexable>
		void set_cell_data_from_indexable(const std::string label, const Indexable& indexable, const size_t begin, const size_t end, const vtkio::AttributeType attr_type);
		template<class TwiceIndexable>
		void set_cell_data_from_twice_indexable(const std::string label, const TwiceIndexable& twice_indexable, const vtkio::AttributeType attr_type);
		template<class TwiceIndexable>
		void set_cell_data_from_twice_indexable(const std::string label, const TwiceIndexable& twice_indexable, const size_t begin, const size_t end, const vtkio::AttributeType attr_type);

		void set_comments(const std::string comments);

		// Getters -----------------------------------------------------------------------
		size_t get_number_of_points();
		size_t get_number_of_cells();
		size_t get_point_data_dimensions(const std::string label);
		size_t get_cell_data_dimensions(const std::string label);
		std::vector<std::string> get_point_data_labels() const;
		std::vector<std::string> get_cell_data_labels() const;
		std::string get_comments();

		template<class T>
		void get_points_raw(std::vector<T>& points);
		template<class TwiceIndexable>
		void get_points_to_twice_indexable(TwiceIndexable& twice_indexable);
		template<class TwiceIndexable>
		void get_points_to_twice_indexable(TwiceIndexable& twice_indexable, const size_t begin, const size_t end);

		template<class T>
		void get_cells_raw(std::vector<T>& cell_array);
		template<class T>
		void get_cell_types_raw(std::vector<T>& cell_types_array);
		template<class TwiceIndexable>
		vtkio::CellType get_cells_to_twice_indexable(TwiceIndexable& twice_indexable);
		template<class TwiceIndexable>
		vtkio::CellType get_cells_to_twice_indexable(TwiceIndexable& twice_indexable, const size_t begin, const size_t end);

		template<class T>
		vtkio::AttributeType get_point_data_raw(const std::string label, std::vector<T>& data);
		template<class Indexable>
		vtkio::AttributeType get_point_data_to_indexable(const std::string label, Indexable& indexable);
		template<class Indexable>
		vtkio::AttributeType get_point_data_to_indexable(const std::string label, Indexable& indexable, const size_t begin, const size_t end);
		template<class TwiceIndexable>
		vtkio::AttributeType get_point_data_to_twice_indexable(const std::string label, TwiceIndexable& twice_indexable);
		template<class TwiceIndexable>
		vtkio::AttributeType get_point_data_to_twice_indexable(const std::string label, TwiceIndexable& twice_indexable, const size_t begin, const size_t end);

		template<class T>
		vtkio::AttributeType get_cell_data_raw(const std::string label, std::vector<T>& data);
		template<class Indexable>
		vtkio::AttributeType get_cell_data_to_indexable(const std::string label, Indexable& indexable);
		template<class Indexable>
		vtkio::AttributeType get_cell_data_to_indexable(const std::string label, Indexable& indexable, const size_t begin, const size_t end);
		template<class TwiceIndexable>
		vtkio::AttributeType get_cell_data_to_twice_indexable(const std::string label, TwiceIndexable& twice_indexable);
		template<class TwiceIndexable>
		vtkio::AttributeType get_cell_data_to_twice_indexable(const std::string label, TwiceIndexable& twice_indexable, const size_t begin, const size_t end);

		// IO -----------------------------------------------------------------------
		void write(const std::string path, const bool binary = true);
		void read(const std::string path, const bool swap_to_local_endianness = true);
		static void write_empty(const std::string path);

		// Other functions -----------------------------------------------------------------------
		void check();
		void clear();
		void swap_bytes(const ByteOrder byte_order = ByteOrder::Native);

		template<class T>
		static VTKFile regular_grid(const std::array<T, 3>& bottom, const std::array<T, 3>& top, const std::array<int, 3>& n_cells_per_dimension);


	private:
		// Auxiliar methods
		size_t _getDataDimensions(const bool point_data, const std::string label);
		VTKData& _getVTKData(const bool point_data, const std::string label);
		template<bool IS_TWICE_INDEXABLE, class Indexable>
		void _setData(const bool point_data, const std::string label, const Indexable& indexable, const size_t begin, const size_t end, const vtkio::AttributeType attr_type);
		template<bool IS_TWICE_INDEXABLE, class Indexable>
		void _getData(const bool point_data, const std::string label, Indexable& indexable, vtkio::AttributeType& attr_type, const size_t begin, const size_t end);
	};


	// Definitions =====================================================================================
	inline void VTKFile::check()
	{
		if (this->comments.find('\n') != std::string::npos) { std::cout << "vtkio error: Comments cannot contain '\n'" << std::endl; exit(-1); }
		if (this->comments.length() > 256) { std::cout << "vtkio error: " << "Comments cannot be larger than 256 characters." << std::endl; exit(-1); }

		// Points
		if (this->get_number_of_points() == 0) { std::cout << "vtkio error: " << "Point array is empty." << std::endl; exit(-1); }
		if (!(this->points.isType<float>() || this->points.isType<double>())) {
			std::cout << "vtkio error: " << "Points must be float or double. Consider using VTKFile.points.castInPlace<double>();" << std::endl; exit(-1);
		};

		// Cells
		if (this->get_number_of_cells() == 0) { std::cout << "vtkio error: " << "Cells array is empty." << std::endl; exit(-1); }
		if (!this->cells.isType<int32_t>()) {
			std::cout << "vtkio error: " << "Cells must be int32_t. Consider using VTKFile.cells.castInPlace<int>();" << std::endl; exit(-1);
		};

		// Data
		auto checkVTKData = [&](const size_t n_entities, std::unordered_map<std::string, VTKData>& data)
		{
			for (auto& key_value : data) {
				std::string label = key_value.first;
				VTKData& vtk_data = key_value.second;
				bytebuffer::ByteBuffer& byte_buffer = vtk_data.buffer;
				const size_t n_data = byte_buffer.getNumberOfItems();
				const size_t dim = n_data / n_entities;
				if (n_data == 0) { std::cout << "vtkio error: " << label + " data array is empty." << std::endl; exit(-1); }
				if (dim * n_entities != n_data) { std::cout << "vtkio error: " << label + " data array dimension mismatchs." << std::endl; exit(-1); }

				const vtkio::AttributeType attr_type = vtk_data.attr_type;
				const vtkio::tables::AttributeTypeInfo attr_info = vtkio::tables::getVTKAttributeTypeInfo(attr_type);
				const std::string attr_type_str = vtkio::tables::getAttributeTypeString(attr_type);
				if (!attr_info.supported) { std::cout << "vtkio error: " << label + " " + attr_type_str + " attribute type not supported." << std::endl; exit(-1); }
				if (dim > attr_info.max_dimensions) { std::cout << "vtkio error: " << label + " " + " has more dimensions than allowed for its attribute type." << std::endl; exit(-1); }
				if (dim < attr_info.min_dimensions) { std::cout << "vtkio error: " << label + " " + " has less dimensions than allowed for its attribute type." << std::endl; exit(-1); }
			}
		};
		checkVTKData(this->get_number_of_points(), this->point_data);
		checkVTKData(this->get_number_of_cells(), this->cell_data);
	}
	inline void VTKFile::clear()
	{
		this->points.clear();
		this->cells.clear();
		this->cell_types.clear();
		this->point_data.clear();
		this->cell_data.clear();
	}
	inline void VTKFile::swap_bytes(const ByteOrder byte_order)
	{
		this->points.swap(byte_order);
		this->cells.swap(byte_order);
		for (auto& label_data : this->point_data) {
			label_data.second.buffer.swap(byte_order);
		}
		for (auto& label_data : this->cell_data) {
			label_data.second.buffer.swap(byte_order);
		}
	}
	inline size_t VTKFile::_getDataDimensions(const bool point_data, const std::string label)
	{
		auto& entity_data = (point_data) ? this->point_data : this->cell_data;
		const size_t n_entities = (point_data) ? this->get_number_of_points() : this->get_number_of_cells();
		auto vtk_data_it = entity_data.find(label);
		if (vtk_data_it == entity_data.end()) { std::cout << "vtkio error: Label not found in data." << std::endl; exit(-1); }
		auto& vtk_data = vtk_data_it->second;
		return vtk_data.buffer.getNumberOfItems() / n_entities;
	}
	inline VTKData& VTKFile::_getVTKData(const bool point_data, const std::string label)
	{
		auto& entity_data = (point_data) ? this->point_data : this->cell_data;
		auto vtk_data_it = entity_data.find(label);
		if (vtk_data_it == entity_data.end()) { std::cout << "vtkio error: Label '" + label + "' not found in data." << std::endl; exit(-1); }
		return vtk_data_it->second;
	}
	inline void VTKFile::set_comments(const std::string comments)
	{
		if (comments.find('\n') != std::string::npos) { std::cout << "vtkio error: Comments cannot contain '\n'" << std::endl; exit(-1); }
		if (comments.length() > 256) { std::cout << "vtkio error: " << "Comments cannot be larger than 256 characters." << std::endl; exit(-1); }
		this->comments = comments;
	}
	inline size_t VTKFile::get_number_of_points()
	{
		return this->points.getNumberOfItems() / 3;
	}
	inline size_t VTKFile::get_number_of_cells()
	{
		return this->cell_types.getNumberOfItems();
	}
	inline std::string VTKFile::get_comments()
	{
		return this->comments;
	}
	inline size_t VTKFile::get_point_data_dimensions(const std::string label)
	{
		return this->_getDataDimensions(true, label);
	}
	inline size_t VTKFile::get_cell_data_dimensions(const std::string label)
	{
		return this->_getDataDimensions(false, label);
	}
	inline std::vector<std::string> VTKFile::get_point_data_labels() const
	{
		std::vector<std::string> labels;
		for (auto it : this->point_data) {
			labels.push_back(it.first);
		}
		return labels;
	}
	inline std::vector<std::string> VTKFile::get_cell_data_labels() const
	{
		std::vector<std::string> labels;
		for (auto it : this->cell_data) {
			labels.push_back(it.first);
		}
		return labels;
	}
	inline void vtkio::VTKFile::write(const std::string path, const bool binary)
	{
		// Input checking
		this->check();

		//// TODO: Check that path is correct and has correct extension. Create folders is necessary.
		//// NOTE: The iteration number is responsibility of the user or another method wrapper
		const std::string dst = path;

		// Open the file
		std::ofstream outfile(dst, std::ios::binary);
		if (!outfile) {
			std::cout << "vtkio error: " << "Cannot open the file " + dst << std::endl; exit(-1);
		}

		// Header
		outfile << "# vtk DataFile Version 4.2\n";
		outfile << this->comments << '\n';
		outfile << ((binary) ? "BINARY\n" : "ASCII\n");
		outfile << "DATASET UNSTRUCTURED_GRID\n";

		// Points
		outfile << "POINTS " << this->get_number_of_points() << " " << vtkio::tables::getVTKDataTypeString(this->points.getDataType()) << "\n";
		if (binary) {
			this->points.toStreamBinary(outfile, ByteOrder::Big);
		}
		else {
			this->points.toStreamASCII(outfile);
		}
		outfile << "\n";

		// Cells
		const size_t n_items = this->cells.getNumberOfItems(); // Includes the n_nodes_per_cell
		outfile << "CELLS " << this->get_number_of_cells() << " " << n_items << "\n";
		if (binary) {
			this->cells.toStreamBinary(outfile, ByteOrder::Big);
		}
		else {
			this->cells.toStreamASCII(outfile);
		}
		outfile << "\n";

		// Cell types
		outfile << "CELL_TYPES " << this->get_number_of_cells() << "\n";
		if (binary) {
			this->cell_types.toStreamBinary(outfile, ByteOrder::Big);
		}
		else {
			this->cell_types.toStreamASCII(outfile);
		}
		outfile << "\n";

		// Point and cell data
		auto saveVTKData = [&](const std::string title, const size_t n_entities, std::unordered_map<std::string, VTKData>& data)
		{
			const size_t num_fields = data.size();
			if (num_fields != 0) {
				outfile << title << " " << n_entities << "\n";

				for (auto& key_value : data) {
					const std::string& label = key_value.first;
					VTKData& vtk_data = key_value.second;
					bytebuffer::ByteBuffer& byte_buffer = vtk_data.buffer;
					const vtkio::AttributeType attr_type = vtk_data.attr_type;
					const vtkio::tables::AttributeTypeInfo attr_info = vtkio::tables::getVTKAttributeTypeInfo(attr_type);
					const std::string attr_type_str = vtkio::tables::getAttributeTypeString(attr_type);

					// Gather data
					const size_t dim = byte_buffer.getNumberOfItems() / n_entities;
					const std::string data_type_str = vtkio::tables::getVTKDataTypeString(byte_buffer.getDataType());

					// Write header
					switch (attr_type)
					{
					case vtkio::AttributeType::Scalars:   outfile << "SCALARS " << label << " " << data_type_str << " " << dim << "\n" << "LOOKUP_TABLE default\n"; break;
					case vtkio::AttributeType::Vectors:   outfile << "VECTORS " << label << " " << data_type_str << "\n"; break;
					case vtkio::AttributeType::Tensors:   outfile << "TENSORS " << label << " " << data_type_str << "\n"; break;
					case vtkio::AttributeType::Normals:   outfile << "NORMALS " << label << " " << data_type_str << "\n"; break;
					case vtkio::AttributeType::FieldData: outfile << "FIELD FieldData 1\n" << label << " " << dim << " " << n_entities << " " << data_type_str << "\n"; break;
					default:
						std::cout << "vtkio error: " << attr_type_str + " AttributeType not supported." << std::endl; exit(-1);
						break;
					}

					// Write data
					if (binary) {
						byte_buffer.toStreamBinary(outfile, ByteOrder::Big);
					}
					else {
						byte_buffer.toStreamASCII(outfile);
					}
					outfile << "\n";
				}
			}
		};

		saveVTKData("POINT_DATA", this->get_number_of_points(), this->point_data);
		saveVTKData("CELL_DATA", this->get_number_of_cells(), this->cell_data);
	}
	inline void VTKFile::write_empty(const std::string path)
	{
		// Open the file
		std::ofstream outfile(path, std::ios::binary);
		if (!outfile) {
			std::cout << "Cannot open a file " << path << " to save a VTK mesh." << std::endl;
			exit(-1);
		}

		// Header
		outfile << "# vtk DataFile Version 4.2\n";
		outfile << "\n";
		outfile << "ASCII\n";
		outfile << "DATASET UNSTRUCTURED_GRID\n";
		outfile << "POINTS 0 double\n";
		outfile << "CELLS 0 0\n";
		outfile << "CELL_TYPES 0\n";

		outfile.close();
	};
	inline void VTKFile::read(const std::string path, const bool swap_to_local_endianness)
	{
		this->clear();

		// Open file
		std::ifstream input(path, std::ios::binary);
		if (!input) {
			std::cout << "Error: Cannot read file " << path << std::endl;
			exit(-1);
		}

		// Initialize buffers
		std::string line_buffer;
		std::string word_buffer;

		// Read header
		bool is_binary = false;
		{
			std::getline(input, line_buffer); // # vtk DataFile Version 4.1
			std::getline(input, line_buffer); // comments
			this->set_comments(line_buffer);
			while (line_buffer.substr(0, 7) != "DATASET") {
				std::getline(input, line_buffer);
				if (line_buffer.substr(0, 5) == "ASCII") {
					is_binary = false;
				}
				else if (line_buffer.substr(0, 6) == "BINARY") {
					is_binary = true;
				}
			}

			// line_buffer is "DATASET" at this point
			if (line_buffer.substr(8, 17) != "UNSTRUCTURED_GRID") {
				std::cout << "vtkio error: Only DATASET UNSTRUCTURED_GRID can be parsed." << std::endl; exit(-1);
			}
		}
		// line_buffer is DATASET at this point

		// Read dataset
		{
			bool read_points = false;
			bool read_cells = false;
			bool read_cell_types = false;
			while (!read_points || !read_cells || !read_cell_types) {
				if (line_buffer.substr(0, 6) == "POINTS") {
					read_points = true;

					// Points header
					std::istringstream line_stream(line_buffer);
					std::string points_word, n_points_str, type_str;
					std::getline(line_stream, points_word, ' ');
					std::getline(line_stream, n_points_str, ' ');
					std::getline(line_stream, type_str);

					DataType points_type;
					if (type_str == "double") {
						points_type = DataType::Float64;
					}
					else if (type_str == "float") {
						points_type = DataType::Float32;
					}
					else {
						std::cout << "vtkio error: Points can only be 'double' or 'float' in VTK." << std::endl; exit(-1);
					}

					// Read the data
					if (is_binary) {
						const size_t n_bytes = getDataTypeSize(points_type) * std::stoi(n_points_str) * 3;
						this->points.setFromBinaryStream(input, points_type, n_bytes, ByteOrder::Big);
					}
					else {
						this->points.setFromASCIIStream(input, points_type, 3 * std::stoi(n_points_str), ' ');
					}
				}
				else if (line_buffer.substr(0, 5) == "CELLS") {
					read_cells = true;

					// Cells header
					std::istringstream line_stream(line_buffer);
					std::string cells_word, n_cells_str, total_n_items_str;
					std::getline(line_stream, cells_word, ' ');
					std::getline(line_stream, n_cells_str, ' ');
					std::getline(line_stream, total_n_items_str);

					// Read the data
					if (is_binary) {
						const size_t n_bytes = getDataTypeSize(DataType::Int32) * std::stoi(total_n_items_str);
						this->cells.setFromBinaryStream(input, DataType::Int32, n_bytes, ByteOrder::Big);
					}
					else {
						this->cells.setFromASCIIStream(input, DataType::Int32, std::stoi(total_n_items_str), ' ');
					}
				}
				else if (line_buffer.substr(0, 10) == "CELL_TYPES") {
					read_cell_types = true;

					// Cells header
					std::istringstream line_stream(line_buffer);
					std::string cell_types_word, n_cells_str;
					std::getline(line_stream, cell_types_word, ' ');
					std::getline(line_stream, n_cells_str, ' ');

					// Read the data
					if (is_binary) {
						const size_t n_bytes = getDataTypeSize(DataType::Int32) * std::stoi(n_cells_str);
						this->cell_types.setFromBinaryStream(input, DataType::Int32, n_bytes, ByteOrder::Big);
					}
					else {
						this->cell_types.setFromASCIIStream(input, DataType::Int32, std::stoi(n_cells_str), ' ');
					}
				}
				std::getline(input, line_buffer);

				// Infinite loop ?
			}
		}
		// At this point the cursor is right after the DATASET

		auto readData = [&](const bool point_data, const bool is_binary, const size_t n_entities, const std::string end_line) {
			auto readDataSeries = [&](const bool point_data, const bool is_binary, const std::string label, const vtkio::AttributeType attr_type, const std::string data_type_str, const std::string dim_str, const size_t n_entities)
			{
				auto& vtk_data = (point_data) ? this->point_data[label] : this->cell_data[label];
				vtk_data.attr_type = attr_type;
				const DataType data_type = vtkio::tables::getDataTypeFromString(data_type_str);
				const size_t n_items_to_read = std::stoi(dim_str) * n_entities;

				if (is_binary) {
					const size_t n_bytes = getDataTypeSize(data_type) * n_items_to_read;
					vtk_data.buffer.setFromBinaryStream(input, data_type, n_bytes, ByteOrder::Big);
				}
				else {
					vtk_data.buffer.setFromASCIIStream(input, data_type, n_items_to_read, ' ');
				}
			};

			auto readFixedDimDataSeries = [&](const bool point_data, const bool is_binary, const std::string header, const vtkio::AttributeType attr_type, const std::string dim_str, const size_t n_entities)
			{
				std::istringstream line_stream(header);
				std::string attr_type_word, label, data_type_str;
				std::getline(line_stream, attr_type_word, ' ');
				std::getline(line_stream, label, ' ');
				std::getline(line_stream, data_type_str);
				readDataSeries(point_data, is_binary, label, attr_type, data_type_str, dim_str, n_entities);
			};

			while (line_buffer.substr(0, end_line.length()) != end_line && !input.eof()) {
				if (line_buffer.substr(0, 7) == "SCALARS") {
					std::istringstream line_stream(line_buffer);
					std::string scalar_word, label, dim_str, data_type_str;
					std::getline(line_stream, scalar_word, ' ');
					std::getline(line_stream, label, ' ');
					std::getline(line_stream, data_type_str, ' ');
					std::getline(line_stream, dim_str);

					std::getline(input, line_buffer);
					//if (line_buffer.substr(0, 20) != "LOOKUP_TABLE default") {
					//	std::cout << "vtkio warning: non-default LOOKUP_TABLE found. This may cause problems reading AttributeType::Scalars." << std::endl;
					//}
					readDataSeries(point_data, is_binary, label, vtkio::AttributeType::Scalars, data_type_str, dim_str, n_entities);
				}
				else if (line_buffer.substr(0, 7) == "VECTORS") {
					readFixedDimDataSeries(point_data, is_binary, line_buffer, vtkio::AttributeType::Vectors, "3", n_entities);
				}
				else if (line_buffer.substr(0, 7) == "TENSORS") {
					readFixedDimDataSeries(point_data, is_binary, line_buffer, vtkio::AttributeType::Tensors, "9", n_entities);
				}
				else if (line_buffer.substr(0, 7) == "NORMALS") {
					readFixedDimDataSeries(point_data, is_binary, line_buffer, vtkio::AttributeType::Normals, "3", n_entities);
				}
				else if (line_buffer.substr(0, 5) == "FIELD") {
					// Note: Each entry in any field data is stripped from the hierarchy and just added in the flat data map

					std::istringstream line_stream(line_buffer);
					std::string field_word, field_data_word, n_fields_str;
					std::getline(line_stream, field_word, ' ');
					std::getline(line_stream, field_data_word, ' ');
					std::getline(line_stream, n_fields_str);

					// Different fields to read
					const size_t n_fields = std::stoi(n_fields_str);
					size_t read_fields = 0;
					while (read_fields < n_fields) {
						std::getline(input, line_buffer);
						if (line_buffer.length() != 0) {
							read_fields++;

							std::istringstream line_stream(line_buffer);
							std::string label, dim_str, n_entities_str, data_type_str;
							std::getline(line_stream, label, ' ');
							std::getline(line_stream, dim_str, ' ');
							std::getline(line_stream, n_entities_str, ' ');
							std::getline(line_stream, data_type_str);

							readDataSeries(point_data, is_binary, label, vtkio::AttributeType::FieldData, data_type_str, dim_str, n_entities);
						}
					}
				}
				std::getline(input, line_buffer);
			}
		};

		// CELL_DATA or POINT_DATA can be first
		while (!input.eof()) {
			if (line_buffer.substr(0, 10) == "POINT_DATA") {
				readData(true, is_binary, this->get_number_of_points(), /* Escape line = */ "CELL_DATA");
			}
			else if (line_buffer.substr(0, 9) == "CELL_DATA") {
				readData(false, is_binary, this->get_number_of_cells(), /* Escape line = */ "POINT_DATA");
			}
			else {
				std::getline(input, line_buffer);
			}
		}

		if (swap_to_local_endianness) {
			this->swap_bytes(endianness::ByteOrder::Native);
		}
		this->check();
	}
	template<class T>
	inline void VTKFile::set_points_raw(const std::vector<T>& points_3d_flat)
	{
		static_assert(std::is_same_v<T, double> || std::is_same_v<T, float>, "Points must be of double or float type.");
		this->points.setFromIndexable(points_3d_flat, 0, points_3d_flat.size());
	}
	template<class TwiceIndexable>
	inline void VTKFile::set_points_from_twice_indexable(const TwiceIndexable& twice_indexable)
	{
		this->set_points_from_twice_indexable(twice_indexable, 0, twice_indexable.size());
	}
	template<class TwiceIndexable>
	inline void VTKFile::set_points_from_twice_indexable(const TwiceIndexable& twice_indexable, const size_t begin, const size_t end)
	{
		if (twice_indexable.size() == 0) { return; }
		using T = typename std::remove_const<typename std::remove_reference<decltype(twice_indexable[0][0])>::type>::type;
		static_assert(std::is_same_v<T, double> || std::is_same_v<T, float>, "Points must be of double or float type.");

		const size_t inner_size = twice_indexable[0].size();
		const size_t n_points = end - begin;
		if ((inner_size == 0) || (inner_size > 3)) { std::cout << "vtkio error: Points dimensions must be 1, 2 or 3." << std::endl; exit(-1); }

		if (inner_size == 3) {
			this->points.setFromTwiceIndexable(twice_indexable, begin, end);
		}
		else {
			// Add zeros in the remaining dimensions
			std::vector<T> points3d;
			points3d.reserve(3 * n_points);
			for (size_t i = begin; i < end; i++) {
				for (size_t j = 0; j < inner_size; j++) {
					points3d.push_back(T(twice_indexable[i][j]));
				}
				for (size_t i = inner_size; i < 3; i++) {
					points3d.push_back(T(0.0));
				}
			}
			this->points.setFromIndexable(points3d);
		}
	}
	template<class TwiceIndexable>
	inline void VTKFile::set_cells_from_twice_indexable(const TwiceIndexable& twice_indexable, const vtkio::CellType cell_type)
	{
		this->set_cells_from_twice_indexable(twice_indexable, 0, twice_indexable.size(), cell_type);
	}
	template<class TwiceIndexable>
	inline void VTKFile::set_cells_from_twice_indexable(const TwiceIndexable& twice_indexable, const size_t begin, const size_t end, const vtkio::CellType cell_type)
	{
		if (twice_indexable.size() == 0) { return; }
		using T = typename std::remove_const<typename std::remove_reference<decltype(twice_indexable[0][0])>::type>::type;

		const size_t inner_size = twice_indexable[0].size();
		const size_t n_cells = end - begin;

		// Build VTK cells array
		std::vector<int32_t> cells;
		cells.reserve((inner_size + 1) * n_cells);
		std::vector<int> cell_types(n_cells, static_cast<int>(cell_type));
		for (size_t i = begin; i < end; i++) {
			cells.push_back((int)inner_size);
			for (size_t j = 0; j < inner_size; j++) {
				cells.push_back(twice_indexable[i][j]);
			}
		}
		this->cell_types.setFromIndexable(cell_types);
		this->cells.setFromIndexable(cells);
	}
	template<class T>
	inline void VTKFile::set_point_data_raw(const std::string label, const std::vector<T>& data_flat, const vtkio::AttributeType attr_type)
	{
		this->_setData<false>(true, label, data_flat, 0, data_flat.size(), attr_type);
	}
	template<class Indexable>
	inline void VTKFile::set_point_data_from_indexable(const std::string label, const Indexable& indexable, const vtkio::AttributeType attr_type)
	{
		this->set_point_data_from_indexable(label, indexable, 0, indexable.size(), attr_type);
	}
	template<class Indexable>
	inline void VTKFile::set_point_data_from_indexable(const std::string label, const Indexable& indexable, const size_t begin, const size_t end, const vtkio::AttributeType attr_type)
	{
		this->_setData<false>(true, label, indexable, begin, end, attr_type);
	}
	template<class TwiceIndexable>
	inline void VTKFile::set_point_data_from_twice_indexable(const std::string label, const TwiceIndexable& twice_indexable, const vtkio::AttributeType attr_type)
	{
		this->set_point_data_from_twice_indexable(label, twice_indexable, 0, twice_indexable.size(), attr_type);
	}
	template<class TwiceIndexable>
	inline void VTKFile::set_point_data_from_twice_indexable(const std::string label, const TwiceIndexable& twice_indexable, const size_t begin, const size_t end, const vtkio::AttributeType attr_type)
	{
		this->_setData<true>(true, label, twice_indexable, begin, end, attr_type);
	}
	template<class T>
	inline void VTKFile::set_cell_data_raw(const std::string label, const std::vector<T>& data_flat, const vtkio::AttributeType attr_type)
	{
		this->_setData<false>(false, label, data_flat, 0, data_flat.size(), attr_type);
	}
	template<class Indexable>
	inline void VTKFile::set_cell_data_from_indexable(const std::string label, const Indexable& indexable, const vtkio::AttributeType attr_type)
	{
		this->set_cell_data_from_indexable(label, indexable, 0, indexable.size(), attr_type);
	}
	template<class Indexable>
	inline void VTKFile::set_cell_data_from_indexable(const std::string label, const Indexable& indexable, const size_t begin, const size_t end, const vtkio::AttributeType attr_type)
	{
		this->_setData<false>(false, label, indexable, begin, end, attr_type);
	}
	template<class TwiceIndexable>
	inline void VTKFile::set_cell_data_from_twice_indexable(const std::string label, const TwiceIndexable& twice_indexable, const vtkio::AttributeType attr_type)
	{
		this->set_cell_data_from_twice_indexable(label, twice_indexable, 0, twice_indexable.size(), attr_type);
	}
	template<class TwiceIndexable>
	inline void VTKFile::set_cell_data_from_twice_indexable(const std::string label, const TwiceIndexable& twice_indexable, const size_t begin, const size_t end, const vtkio::AttributeType attr_type)
	{
		this->_setData<true>(false, label, twice_indexable, begin, end, attr_type);
	}
	template<class T>
	inline void VTKFile::get_points_raw(std::vector<T>& points)
	{
		this->points.getToIndexable(points);
	}
	template<class TwiceIndexable>
	inline void VTKFile::get_points_to_twice_indexable(TwiceIndexable& twice_indexable)
	{
		twice_indexable.resize(this->get_number_of_points());
		this->get_points_to_twice_indexable(twice_indexable, 0, twice_indexable.size());
	}
	template<class TwiceIndexable>
	inline void VTKFile::get_points_to_twice_indexable(TwiceIndexable& twice_indexable, const size_t begin, const size_t end)
	{
		if (end - begin != this->get_number_of_points()) { std::cout << "vtkio error: Points size mismatch." << std::endl; exit(-1); }
		if (twice_indexable[begin].size() == 0) { std::cout << "vtkio error: Inner container of TwiceIndexable has size 0" << std::endl; exit(-1); }

		using T = typename std::remove_reference<decltype(twice_indexable[0][0])>::type;
		this->points.throwIfNotType<T>();

		const size_t inner_size = twice_indexable[0].size();
		T* points_it = this->points.getDataPtr<T>();
		for (size_t i = begin; i < end; i++) {
			for (size_t j = 0; j < inner_size; j++) {
				twice_indexable[i][j] = *points_it;
				points_it++;
			}
			for (size_t j = inner_size; j < 3; j++) {
				points_it++;
			}
		}
	}
	template<class T>
	inline void VTKFile::get_cells_raw(std::vector<T>& cell_array)
	{
		this->cells.getToIndexable(cell_array);
	}
	template<class T>
	inline void VTKFile::get_cell_types_raw(std::vector<T>& cell_types_array)
	{
		this->cell_types.getToIndexable(cell_types_array);
	}
	template<class TwiceIndexable>
	inline vtkio::CellType VTKFile::get_cells_to_twice_indexable(TwiceIndexable& twice_indexable)
	{
		twice_indexable.resize(this->get_number_of_cells());
		return this->get_cells_to_twice_indexable(twice_indexable, 0, twice_indexable.size());
	}
	template<class TwiceIndexable>
	inline vtkio::CellType VTKFile::get_cells_to_twice_indexable(TwiceIndexable& twice_indexable, const size_t begin, const size_t end)
	{
		const size_t n_cells = this->get_number_of_cells();
		if (end - begin != n_cells) { std::cout << "vtkio error: Cell size mismatch." << std::endl; exit(-1); }
		if (twice_indexable[begin].size() == 0) { std::cout << "vtkio error: Inner container of TwiceIndexable has size 0. Probably due to being dynamically sized. Consider using a statically sized inner container instead (such as std::array)." << std::endl; exit(-1); }

		using T = typename std::remove_reference<decltype(twice_indexable[0][0])>::type;
		this->cells.throwIfNotType<T>();

		// Check all cells are the same type
		this->cell_types.swap(endianness::ByteOrder::Native);
		int32_t* cell_type_it = this->cell_types.getDataPtr<int32_t>();
		const int32_t cell_type_0 = cell_type_it[0];
		for (size_t i = 0; i < this->cell_types.getNumberOfItems(); i++) {
			if (cell_type_0 != *cell_type_it) { std::cout << "vtkio error: Cannot use .get_cells_to_twice_indexable() when the file contains different cell types." << std::endl; exit(-1); }
		}

		const size_t inner_size = twice_indexable[0].size();
		int32_t* cells_it = this->cells.getDataPtr<int32_t>();

		for (size_t i = 0; i < n_cells; i++) {
			const size_t nodes_per_cell = *cells_it;
			if (nodes_per_cell != inner_size) { std::cout << "vtkio error: Cell size mismatch." << std::endl; exit(-1); }
			cells_it++;
			for (size_t j = 0; j < nodes_per_cell; j++) {
				twice_indexable[i][j] = *cells_it;
				cells_it++;
			}
		}

		return static_cast<CellType>(cell_type_0);
	}
	template<class T>
	inline vtkio::AttributeType VTKFile::get_point_data_raw(const std::string label, std::vector<T>& data)
	{
		return this->get_point_data_to_indexable(label, data);
	}
	template<class Indexable>
	inline vtkio::AttributeType VTKFile::get_point_data_to_indexable(const std::string label, Indexable& indexable)
	{
		VTKData& vtk_data = this->_getVTKData(true, label);
		indexable.resize(vtk_data.buffer.getNumberOfItems());
		return this->get_point_data_to_indexable(label, indexable, 0, indexable.size());
	}
	template<class Indexable>
	inline vtkio::AttributeType VTKFile::get_point_data_to_indexable(const std::string label, Indexable& indexable, const size_t begin, const size_t end)
	{
		vtkio::AttributeType attr_type_out;
		this->_getData<false>(true, label, indexable, attr_type_out, begin, end);
		return attr_type_out;
	}
	template<class TwiceIndexable>
	inline vtkio::AttributeType VTKFile::get_point_data_to_twice_indexable(const std::string label, TwiceIndexable& twice_indexable)
	{
		VTKData& vtk_data = this->_getVTKData(true, label);
		twice_indexable.resize(this->get_number_of_points());
		return this->get_point_data_to_twice_indexable(label, twice_indexable, 0, twice_indexable.size());
	}
	template<class TwiceIndexable>
	inline vtkio::AttributeType VTKFile::get_point_data_to_twice_indexable(const std::string label, TwiceIndexable& twice_indexable, const size_t begin, const size_t end)
	{
		vtkio::AttributeType attr_type_out;
		this->_getData<true>(true, label, twice_indexable, attr_type_out, begin, end);
		return attr_type_out;
	}
	template<class T>
	inline vtkio::AttributeType VTKFile::get_cell_data_raw(const std::string label, std::vector<T>& data)
	{
		return this->get_cell_data_to_indexable(label, data);
	}
	template<class Indexable>
	inline vtkio::AttributeType VTKFile::get_cell_data_to_indexable(const std::string label, Indexable& indexable)
	{
		VTKData& vtk_data = this->_getVTKData(false, label);
		indexable.resize(vtk_data.buffer.getNumberOfItems());
		return this->get_cell_data_to_indexable(label, indexable, 0, indexable.size());
	}
	template<class Indexable>
	inline vtkio::AttributeType VTKFile::get_cell_data_to_indexable(const std::string label, Indexable& indexable, const size_t begin, const size_t end)
	{
		vtkio::AttributeType attr_type_out;
		this->_getData<false>(false, label, indexable, attr_type_out, begin, end);
		return attr_type_out;
	}
	template<class TwiceIndexable>
	inline vtkio::AttributeType VTKFile::get_cell_data_to_twice_indexable(const std::string label, TwiceIndexable& twice_indexable)
	{
		VTKData& vtk_data = this->_getVTKData(false, label);
		twice_indexable.resize(this->get_number_of_cells());
		return this->get_cell_data_to_twice_indexable(label, twice_indexable, 0, twice_indexable.size());
	}
	template<class TwiceIndexable>
	inline vtkio::AttributeType VTKFile::get_cell_data_to_twice_indexable(const std::string label, TwiceIndexable& twice_indexable, const size_t begin, const size_t end)
	{
		vtkio::AttributeType attr_type_out;
		this->_getData<true>(false, label, twice_indexable, attr_type_out, begin, end);
		return attr_type_out;
	}
	inline void VTKFile::set_cells_raw(const std::vector<int>& cell_array_flat)
	{
		this->cells.setFromIndexable(cell_array_flat);
	}
	inline void VTKFile::set_cell_types_raw(const std::vector<int>& cell_type_array_flat)
	{
		this->cell_types.setFromIndexable(cell_type_array_flat);
	}
	inline void VTKFile::set_cells_as_particles(const size_t n_particles)
	{
		if (n_particles == 0) { return; }
		std::vector<int32_t> cells;
		cells.reserve(2 * n_particles);
		for (int32_t i = 0; i < n_particles; i++) {
			cells.push_back(1);
			cells.push_back(i);
		}
		std::vector<int> cell_types(n_particles, static_cast<int>(vtkio::CellType::Vertex));
		this->cell_types.setFromIndexable(cell_types);
		this->cells.setFromIndexable(cells);
	}
	template<class T>
	inline VTKFile VTKFile::regular_grid(const std::array<T, 3>& bottom, const std::array<T, 3>& top, const std::array<int, 3>& n_cells_per_dimension)
	{
		// Grid
		const int cx = n_cells_per_dimension[0];
		const int cy = n_cells_per_dimension[1];
		const int cz = n_cells_per_dimension[2];
		const int nx = cx + 1;
		const int ny = cy + 1;
		const int nz = cz + 1;
		const int n_points = nx * ny * nz;
		const int n_cells = cx * cy * cz;
		const T dx = (top[0] - bottom[0]) / (T)cx;
		const T dy = (top[1] - bottom[1]) / (T)cy;
		const T dz = (top[2] - bottom[2]) / (T)cz;

		// Points
		/* Correspond to a hexahedron following the pattern:
				0 -> [[0, 0, 0],
				1 ->  [0, 0, 1],
				2 ->  [0, 1, 0],
				3 ->  [0, 1, 1],
				4 ->  [1, 0, 0],
				5 ->  [1, 0, 1],
				6 ->  [1, 1, 0],
				7 ->  [1, 1, 1]]

			move z -> move y -> move x
		*/
		std::vector<std::array<T, 3>> vertices(n_points);
		for (int i = 0; i < nx; i++) {
			for (int j = 0; j < ny; j++) {
				for (int k = 0; k < nz; k++) {
					vertices[nz * ny * i + nz * j + k] = { bottom[0] + i * dx, bottom[1] + j * dy, bottom[2] + k * dz };
				}
			}
		}

		// Connectivity
		std::vector<std::array<int, 8>> hexas(n_cells);
		int c = 0;
		for (int i = 0; i < cx; i++) {
			for (int j = 0; j < cy; j++) {
				for (int k = 0; k < cz; k++) {
					std::array<int, 8> nodes = { nz * ny * (i + 0) + nz * (j + 0) + (k + 0),
												 nz * ny * (i + 0) + nz * (j + 0) + (k + 1),
												 nz * ny * (i + 0) + nz * (j + 1) + (k + 0),
												 nz * ny * (i + 0) + nz * (j + 1) + (k + 1),
												 nz * ny * (i + 1) + nz * (j + 0) + (k + 0),
												 nz * ny * (i + 1) + nz * (j + 0) + (k + 1),
												 nz * ny * (i + 1) + nz * (j + 1) + (k + 0),
												 nz * ny * (i + 1) + nz * (j + 1) + (k + 1) };
					hexas[c] = { nodes[0], nodes[4], nodes[6], nodes[2], nodes[1], nodes[5], nodes[7], nodes[3] };
					c++;
				}
			}
		}

		// VTKFile
		VTKFile vtk_file;
		vtk_file.set_points_from_twice_indexable(vertices);
		vtk_file.set_cells_from_twice_indexable(hexas, vtkio::CellType::Hexahedron);
		return vtk_file;
	}
	template<bool IS_TWICE_INDEXABLE, class Indexable>
	inline void VTKFile::_setData(const bool point_data, const std::string label, const Indexable& indexable, const size_t begin, const size_t end, const vtkio::AttributeType attr_type)
	{
		const size_t n_entities = (point_data) ? this->get_number_of_points() : this->get_number_of_cells();
		const size_t n = end - begin;
		const size_t dim = n / n_entities;
		const size_t max_dim = vtkio::tables::getVTKAttributeTypeInfo(attr_type).max_dimensions;

		if (n == 0) { std::cout << "vtkio error: Data input is empty." << std::endl; exit(-1); }
		if ((dim == 0) || (dim > max_dim)) { std::cout << "vtkio error: " << "Data attribute (" + std::to_string(static_cast<int>(attr_type)) + " dimension must be <= " + std::to_string(max_dim) << std::endl; exit(-1); }
		if (n_entities * dim != n) { std::cout << "vtkio error: Data length and dimensions mismatch with the number of points or cells." << std::endl; exit(-1); }

		auto& vtk_data = (point_data) ? this->point_data[label] : this->cell_data[label];
		vtk_data.attr_type = attr_type;
		if constexpr (IS_TWICE_INDEXABLE) {
			vtk_data.buffer.setFromTwiceIndexable(indexable, begin, end);
		}
		else {
			vtk_data.buffer.setFromIndexable(indexable, begin, end);
		}
	}
	template<bool IS_TWICE_INDEXABLE, class Indexable>
	inline void VTKFile::_getData(const bool point_data, const std::string label, Indexable& indexable, vtkio::AttributeType& attr_type, const size_t begin, const size_t end)
	{
		VTKData& vtk_data = this->_getVTKData(point_data, label);

		if constexpr (IS_TWICE_INDEXABLE) {
			using T = typename std::remove_reference<decltype(indexable[0][0])>::type;
			vtk_data.buffer.throwIfNotType<T>();
		}
		else {
			using T = typename std::remove_reference<decltype(indexable[0])>::type;
			vtk_data.buffer.throwIfNotType<T>();
		}

		attr_type = vtk_data.attr_type;
		if constexpr (IS_TWICE_INDEXABLE) {
			vtk_data.buffer.getToTwiceIndexable(indexable, begin, end);
		}
		else {
			vtk_data.buffer.getToIndexable(indexable, begin, end);
		}
	}
}


