//=============================================================================

#include "BIM_file.h"

#define BIM_FILE_VERSION 1

#include <fstream>
#include <iostream>

//=============================================================================

namespace bim {

//=============================================================================


BIM_file::BIM_file()
{
    pipeline_version_ = "no_version_available";
}

//-----------------------------------------------------------------------------

BIM_file::~BIM_file()
{
    clear();
}

//-----------------------------------------------------------------------------

void BIM_file::clear()
{
    /*
    std::map<std::string, BIM_mesh*>::iterator bmesh_it;
    for (bmesh_it=meshes_.begin(); bmesh_it != meshes_.end(); ++bmesh_it)
    {
        delete bmesh_it->second;
    }
    */
    meshes_.clear();
    skeleton_.clear();
}

//-----------------------------------------------------------------------------

bool BIM_file::read(const char *filename)
{
    filename_ = filename;

    clear();

    std::ifstream ifs(filename, std::ios_base::binary);

    if (!ifs)
        return false;

    int version=-1;
    unsigned int data_size;
    std::string str;
    long int pos,end_pos;

    str.resize(11);
    ifs.read(&str[0], str.size());
    ifs.read(reinterpret_cast<char*>(&version), sizeof(int));

    if (str != "bim_version" || version != 1)
        return false;


    while (ifs.good())
    {
        data_size = read_semantic(ifs, str);


        if (!ifs.good())
            break;

        pos = ifs.tellg();

        if (str.find("type") == 0)
        {
            end_pos = data_size + ifs.tellg();
            if (str == "type skeleton")
            {
                read_skeleton(ifs, end_pos);
            }
            else if (str.find("type mesh") == 0)
            {
                read_mesh(ifs, end_pos, str.substr(10));
            }
            else if (str == "type blendshapes")
            {
                read_blendshapes(ifs, end_pos);
            }
            else if (str == "type pipeline_version")
            {
                read_str(ifs, data_size, pipeline_version_);
            }
        }


        if (pos == ifs.tellg())
        {
            //discard unknown data type
            std::cout << "jump " << str << std::endl;
            ifs.seekg(data_size, ifs.cur);
        }
    }


    ifs.close();

    return true;
}

//-----------------------------------------------------------------------------

bool BIM_file::write(const char *filename)
{

    size_t i,j;
    std::string str;
    std::ofstream ofs(filename, std::ios_base::binary);

    if (!ofs)
        return false;

    unsigned int len;
    long int p0,p1,p2;


    //write version
    str = "bim_version";
    ofs.write(str.c_str(), str.size());
    int version = BIM_FILE_VERSION;
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(int));

    //write skeleton
    if (skeleton_.joints_.size() == skeleton_.parents_.size() && skeleton_.transforms_.size() == skeleton_.joints_.size())
    {
        len = 0;
        p0 = ofs.tellp();
        write_type(ofs, "type skeleton", len);//write placeholder for length
        p1 = ofs.tellp();


        for (i=0; i < skeleton_.joints_.size(); ++i)
        {
            write_str(ofs, "string joint_name", skeleton_.joints_[i]);
            write_str(ofs, "string parent_name", skeleton_.parents_[i]);
            write_vecT(ofs, "float16 local_transform", skeleton_.transforms_[i]);
        }


        p2 = ofs.tellp();
        len = p2-p1;
        ofs.seekp(p0, ofs.beg); //jump back to placeholder
        write_type(ofs, "type skeleton", len);//write real length
        ofs.seekp(len, ofs.cur);//jump back to end
    }


    //write meshes/skins
    std::map< std::string, BIM_mesh >::const_iterator   bmesh_it;
    std::map< std::string, std::string>::const_iterator data_it;
    for (bmesh_it=meshes_.begin(); bmesh_it != meshes_.end(); ++bmesh_it)
    {
        const BIM_mesh &bmesh = bmesh_it->second;


        len = 0;
        p0 = ofs.tellp();
        write_type(ofs, "type mesh " + bmesh_it->first, len);//write placeholder for length
        p1 = ofs.tellp();


        for (data_it = bmesh.semantic_to_data_.begin(); data_it != bmesh.semantic_to_data_.end(); ++data_it)
        {
            const std::string &semantic = data_it->first;
            const std::string &data     = data_it->second;
            write_str(ofs, semantic, data);
        }

        p2 = ofs.tellp();
        len = p2-p1;
        ofs.seekp(p0, ofs.beg); //jump back to placeholder
        write_type(ofs, "type mesh " + bmesh_it->first, len);//write real length
        ofs.seekp(len, ofs.cur);//jump back to end
    }


    for (i=0; i < blendshapes_.size(); ++i)
    {
        const BIM_blendshapes& bim_bs = blendshapes_[i];

        len = 0;
        p0 = ofs.tellp();
        write_type(ofs, "type blendshapes", len);//write placeholder for length
        p1 = ofs.tellp();

        write_str(ofs, "string base_id", bim_bs.base_id_);
        for (j=0; j < bim_bs.target_ids_.size(); ++j)
        {
            write_str(ofs, "string target_id", bim_bs.target_ids_[j]);
        }

        p2 = ofs.tellp();
        len = p2-p1;
        ofs.seekp(p0, ofs.beg); //jump back to placeholder
        write_type(ofs, "type blendshapes", len);//write real length
        ofs.seekp(len, ofs.cur);//jump back to end
    }


    ofs.close();

    return true;
}


//-----------------------------------------------------------------------------


void BIM_file::read_skeleton(std::istream &is, long int end_pos)
{
    unsigned int data_size;
    long int pos;

    std::string semantic_string;
    std::string str;
    std::vector<float> lcl_transform;

    while (is.good())
    {
        data_size = read_semantic(is, semantic_string);

        if (!is.good())
            break;

        pos = is.tellg();

        bool empty_parent_name = false; // Ok for root

        if (semantic_string == "string joint_name")
        {
            read_str(is, data_size, str);
            skeleton_.joints_.push_back(str);
        }
        else if (semantic_string == "string parent_name")
        {
            read_str(is, data_size, str);
            skeleton_.parents_.push_back(str);

            if (str.size() == 0)
                empty_parent_name = true;
        }
        else if (semantic_string == "float16 local_transform")
        {
            read_vecT(is, data_size, lcl_transform);
            skeleton_.transforms_.push_back(lcl_transform);
        }

        //
        // We either did not match semantic_string to a known value
        // or read an empty item.
        // This is only ok for the root joint, which has an empty
        // parent_name
        //
        if (pos == is.tellg() && !empty_parent_name)
        {
            //discard unknown data type
            std::cout << "unexpected data in read_skeleton: " << semantic_string << std::endl;
            is.seekg(data_size, is.cur);
        }

        if (is.tellg() >= end_pos)
            break;
    }

}

//-----------------------------------------------------------------------------

void BIM_file::read_mesh(std::istream &is, long end_pos, const std::string& mesh_id)
{
    unsigned int data_size;
    long int pos;

    std::string str;

    BIM_mesh &mesh = meshes_[mesh_id];

    while (is.good())
    {
        data_size = read_semantic(is, str);

        if (!is.good())
            break;

        pos = is.tellg();

        std::string &data = mesh.semantic_to_data_[str];
        read_str(is, data_size, data);

        if (pos == is.tellg())
        {
            //discard unknown data type
            std::cout << "jump " << str << std::endl;
            is.seekg(data_size, is.cur);
        }

        if (is.tellg() >= end_pos)
            break;
    }
}


//-----------------------------------------------------------------------------

void BIM_file::read_blendshapes(std::istream &is, long int end_pos)
{
    unsigned int data_size;
    long int pos;

    std::string str;

    blendshapes_.push_back(BIM_blendshapes());
    BIM_blendshapes& bim_bs = blendshapes_.back();

    while (is.good())
    {
        data_size = read_semantic(is, str);

        if (!is.good())
            break;

        pos = is.tellg();


        if (str == "string target_id")
        {
            read_str(is, data_size, str);
            bim_bs.target_ids_.push_back(str);
        }
        else if (str == "string base_id")
        {
            read_str(is, data_size, str);
            bim_bs.base_id_ = str;
        }



        if (pos == is.tellg())
        {
            //discard unknown data type
            std::cout << "jump " << str << std::endl;
            is.seekg(data_size, is.cur);
        }

        if (is.tellg() >= end_pos)
            break;
    }
}

//=============================================================================
} // namespace bim
//=============================================================================
