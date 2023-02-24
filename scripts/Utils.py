import numpy as np


def dict_from_DetectionInfo(ob_data):
    '''A function to create python dictionary form DetectionInfo message of find_object_2d
        parameters: ob_data: object data received from find_object_2d
        return: dict: dictionary with object details'''

    ob_data_dict = []

    for ob in ob_data:
        ob_dict = {'id': ob[0].data,
                   'width': ob[1].data,
                   'height': ob[2].data,
                   'filePath': ob[3].data,
                   'inliers': ob[4].data,
                   'outliers': ob[5].data,
                   'homography': ob[6].data}

        ob_data_dict.append(ob_dict)

    return ob_data_dict


def get_ob_name(ob_filePath):
    '''function to return the name of the object from object sample filePath.  
    Parameters: ob_filePath->String-> object filePath as returned by find_object_2d in 'info' topic 
    returns: string '''
    
    ob_filePath = ob_filePath.split('/')
    ob_name = ob_filePath[8]
    return ob_name
