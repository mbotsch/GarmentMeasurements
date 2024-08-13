#ifndef BIM_FILE_H
#define BIM_FILE_H

#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <cstring>

namespace bim
{


class BIM_file
{
public:
    struct BIM_mesh
    {
        //maps semantic ("what type of data") to the corresponding data in byte (not necessary an ascii string)
        std::map<std::string, std::string > semantic_to_data_;
    };

    struct BIM_skeleton
    {
        std::vector<std::string> joints_;
        std::vector<std::string> parents_;
        std::vector< std::vector<float> > transforms_;

        void clear()
        {
            joints_.clear();
            parents_.clear();
            transforms_.clear();
        }
    };

    struct BIM_blendshapes
    {
        std::string base_id_;
        std::vector<std::string> target_ids_;
    };


public:
    std::string filename_;
    std::string pipeline_version_;

    std::map<std::string, BIM_mesh> meshes_;

    BIM_skeleton skeleton_;

    std::vector<BIM_blendshapes> blendshapes_;

public:

    BIM_file();
    virtual ~BIM_file();

    void clear();

    virtual bool read(const char* filename);

    virtual bool write(const char* filename);

    void add_mesh_data(const std::string& mesh_id, const std::string& semantic, size_t byte_size, const void* byte_data);

    template<class T> static void memcpy_vecT(std::vector<T> &dst, const std::string &src);

private:
    void read_skeleton(std::istream& is, long int end_pos);
    void read_mesh(std::istream& is, long int end_pos, const std::string &mesh_id);
    void read_blendshapes(std::istream& is, long int end_pos);


private:
    unsigned int read_semantic(std::istream& is, std::string& inout_str);
    void read_str(std::istream& is, unsigned int size, std::string& str);
    template <class T> void read_T(std::istream& is, T& t);
    template <class T> void read_vecT(std::istream& is, unsigned int size, std::vector<T>& t);

    void write_type(std::ostream &os, const std::string &semantic, unsigned int length);
    void write_str(std::ostream& os, const std::string& semantic, const std::string& str);
    template<class T> void write_T(std::ostream& os, const std::string& semantic, T value);
    template<class T> void write_vecT(std::ostream& os, const std::string& semantic, const std::vector<T>& vec);

};

//-----------------------------------------------------------------------------

template<class T>
void
BIM_file::memcpy_vecT(std::vector<T> &dst, const std::string &src)
{
    dst.clear();
    dst.resize(src.size()/sizeof(T));
    memcpy(&dst[0], src.c_str(), src.size());
}

//-----------------------------------------------------------------------------

inline void BIM_file::add_mesh_data(const std::string &mesh_id, const std::string &semantic, size_t byte_size, const void *byte_data)
{
    BIM_mesh &mesh = meshes_[mesh_id];

    std::string &bytes = mesh.semantic_to_data_[semantic];

    bytes.resize(byte_size);
    memcpy(&bytes[0], byte_data, bytes.size());
}

//-----------------------------------------------------------------------------

inline unsigned int BIM_file::read_semantic(std::istream &is, std::string &inout_str)
{
    unsigned int data_size=0;
    is.read(reinterpret_cast<char*>(&data_size), sizeof(unsigned int));

    inout_str.resize(data_size);
    is.read(&inout_str[0], data_size);

    data_size = 0;
    is.read(reinterpret_cast<char*>(&data_size), sizeof(unsigned int));

    return data_size;
}

//-----------------------------------------------------------------------------

inline void BIM_file::read_str(std::istream &is, unsigned int size, std::string &str)
{
    str.clear();
    str.resize(size);
    is.read(&str[0], str.size());
}

//-----------------------------------------------------------------------------

template <class T>
inline void BIM_file::read_T(std::istream& is, T& t)
{
    is.read(reinterpret_cast<char*>(&t), sizeof(T));
}

//-----------------------------------------------------------------------------

template <class T>
inline void BIM_file::read_vecT(std::istream& is, unsigned int size, std::vector<T>& t)
{
    t.clear();
    t.resize(size / sizeof(T));
    is.read(reinterpret_cast<char*>(&t[0]), size);
}


//-----------------------------------------------------------------------------

inline void BIM_file::write_type(std::ostream &os, const std::string &semantic, unsigned int length)
{
    unsigned int size;

    size = (unsigned int) semantic.size();
    os.write(reinterpret_cast<const char*>(&size), sizeof(unsigned int));
    os.write(semantic.c_str(), semantic.size());

    os.write(reinterpret_cast<const char*>(&length), sizeof(unsigned int));
}

//-----------------------------------------------------------------------------

inline void BIM_file::write_str(std::ostream &os, const std::string &semantic, const std::string &str)
{
    unsigned int size;

    size = (unsigned int) semantic.size();
    os.write(reinterpret_cast<const char*>(&size), sizeof(unsigned int));
    os.write(semantic.c_str(), semantic.size());

    size = (unsigned int) str.size();
    os.write(reinterpret_cast<const char*>(&size), sizeof(unsigned int));
    os.write(str.c_str(), str.size());
}

//-----------------------------------------------------------------------------

template <class T>
inline void BIM_file::write_T(std::ostream& os, const std::string& semantic, T value)
{
    unsigned int size;

    size = (unsigned int) semantic.size();
    os.write(reinterpret_cast<const char*>(&size), sizeof(unsigned int));
    os.write(semantic.c_str(), semantic.size());

    size = (unsigned int) sizeof(T);
    os.write(reinterpret_cast<const char*>(&size), sizeof(unsigned int));

    os.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

//-----------------------------------------------------------------------------

template <class T>
inline void BIM_file::write_vecT(std::ostream& os, const std::string& semantic, const std::vector<T>& vec)
{
    unsigned int size;

    size = (unsigned int) semantic.size();
    os.write(reinterpret_cast<const char*>(&size), sizeof(unsigned int));
    os.write(semantic.c_str(), semantic.size());

    size = (unsigned int) vec.size() * sizeof(T);
    os.write(reinterpret_cast<const char*>(&size), sizeof(unsigned int));

    os.write(reinterpret_cast<const char*>(vec.data()), size);
}


} //namespace bim

#endif
