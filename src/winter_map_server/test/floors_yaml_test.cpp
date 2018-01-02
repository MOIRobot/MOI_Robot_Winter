#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <vector>

#include "yaml-cpp/yaml.h"


#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

void resolve_yaml(std::string& fname)
{
  std::ifstream fin(fname.c_str());
  int floor_num;
  int lift_num;
  int negate;
  double occ_th, free_th;
  double res;
  
#ifdef HAVE_NEW_YAMLCPP
    // The document loading process changed in yaml-cpp 0.5.
  YAML::Node doc = YAML::Load(fin);
#else
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
#endif
        try { 
          doc["floor_num"] >> floor_num; 
          std::cout << "\nfloor_num:" <<floor_num << std::endl;
        } catch (YAML::InvalidScalar) { 
          printf("The map does not contain a resolution tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["lift_num"] >> lift_num; 
          std::cout << "\nlift_num:" <<lift_num << std::endl;
        } catch (YAML::InvalidScalar) { 
          printf("The map does not contain a lift_num tag or it is invalid.");
          exit(-1);
        }
        
        std::string mapfname[floor_num];
        double origin[floor_num][3];
        double init_pose[floor_num][lift_num][3];
  
        try { 
          doc["resolution"] >> res; 
          std::cout << "\nres:" <<res << std::endl;
        } catch (YAML::InvalidScalar) { 
          printf("The map does not contain a resolution tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["negate"] >> negate; 
          std::cout << "\nnegate:" <<negate << std::endl;
        } catch (YAML::InvalidScalar) { 
          printf("The map does not contain a negate tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["occupied_thresh"] >> occ_th; 
          std::cout << "\nocc_th:" << occ_th << std::endl;
        } catch (YAML::InvalidScalar) { 
          printf("The map does not contain an occupied_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["free_thresh"] >> free_th; 
          std::cout << "\nfree_th:" << free_th << std::endl;
        } catch (YAML::InvalidScalar) { 
          printf("The map does not contain a free_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          std::string modeS = "";
          doc["mode"] >> modeS;
          
        } catch (YAML::Exception) { 
          printf("The map does not contain a mode tag or it is invalid... assuming Trinary");
        }
        try { 
          for(int i = 0; i < floor_num; i++)
          {
            printf("\n");
            for(int j = 0; j < 3;j++){
              doc["origin"][i][j] >> origin[i][j];
              printf(" origin[%d][%d]:%f",i, j, origin[i][j]);
            }
            
          }
        } catch (YAML::InvalidScalar) { 
          printf("The map does not contain an origin tag or it is invalid.");
          exit(-1);
        }
        try { 
          for(int i = 0; i < floor_num; i++)
          {
            doc["image"][i] >> mapfname[i]; 
            // TODO: make this path-handling more robust
            std::cout << "\nmap_name:" << mapfname[i] << std::endl;
          }
        } catch (YAML::InvalidScalar) { 
          printf("The map does not contain an image tag or it is invalid.");
          exit(-1);
        }
         try { 
          for(int i = 0; i < floor_num; i++){
			  for(int j = 0;j< lift_num; j++){
				doc["init_pose"][i][j][0] >> init_pose[i][j][0]; 
				doc["init_pose"][i][j][1] >> init_pose[i][j][1]; 
				doc["init_pose"][i][j][2] >> init_pose[i][j][2]; 
				printf("\ninit_pose[%d][%d][0]: %f ", i, j ,init_pose[i][j][0]);
				printf(" init_pose[%d][%d][1]: %f ", i, j ,init_pose[i][j][1]);
				printf(" init_pose[%d][%d][2]: %f ", i, j ,init_pose[i][j][2]);
				
		    }
          }
        } catch (YAML::InvalidScalar) { 
          printf("The map does not contain an init_pos tag or it is invalid.");
          exit(-1);
        }
  

}

int main(int argc, char **argv)
{
  if (argc != 2) {
    printf("Usage:./read_yaml + name.yaml\n");
  }
  std::string fname(argv[1]);

  try
  {
    resolve_yaml(fname);
  }
  catch(std::runtime_error& e)
  {
    printf("Error\n");
    return -1;
  }

  return 0;
}



























