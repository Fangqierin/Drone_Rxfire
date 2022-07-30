# from dash.dependencies import Input, Output, State
# import dash_table
# import dash_core_components as dcc
# import dash_html_components as html
# import dash
# #import dash_bootstrap_components as dbc
# import dash_bio as dashbio
# from dash_bio.utils import PdbParser, create_mol3d_style
# from dash.dependencies import Input, Output
# import pandas as pd
# from collections import defaultdict
# import plotly.express as px
# import matplotlib.pyplot as plt
# import mpld3 
import pymongo
from pymongo import MongoClient
from bson import ObjectId
#from fire_sim import Sim_fire2
import random

if __name__ == '__main__':
    # floor_num=12
    # va=3
    #rooms_d=pd.read_csv('../data/rooms.csv',sep=',')
    #room_set=list(rooms_d['r'])
    #room_AF=dict(zip(list(rooms_d['r']),(zip(list(rooms_d['w']),list(rooms_d['d'])))))
    #all_room=len(room_AF)*floor_num
    # ii=0
    # time_slot=1
    # random.seed(ii)
    # a=random.sample(range(all_room),va) #fire source 
    # #print(f"fire source",a)
    # output=f"./result/dash/sim_{ii}.csv"
    #Sim,Sim_real,fire_floors,hum_floors,win_floors=Get_sim(0,60,ii,a,output,time_slot)
    #client = MongoClient("mongodb://169.234.7.170:27017/")
    from mongotriggers import MongoTrigger 
    client = MongoClient("mongodb://169.234.54.191:27017/")
    #print(client.list_database_names())
    # Create database called animals
    #mydb = client["images"]
    
    def trigger_function(op_document):
        print("trigger_image_processing called")
        print(op_document)
    
    triggers = MongoTrigger(client)

    triggers.register_insert_trigger(trigger_function,"Command", 'Simfire')

    triggers.tail_oplog()
    mydb = client["Command"]
    # Create Collection (table) called shelterA
    collection = mydb.Simfire
    #collection = mydb.currentImages
    print(collection)
    cursor=list(collection.find({}))
    print(cursor)
    for document in cursor:
        print(f"see", document)
        
    #windows=pd.read_csv('../data/all_win.csv',sep=' ')
    #collection.delete_many({})
    # for key, item in Sim.items():
    #     for index, row in windows.iterrows():
    #             #print(windows.columns)
    #     #         print(row._get_value('id'))
    #     #         print(row['id'])
    #             if row["id"]in item['f']:
    #                 record={"time": key, "id":row["id"], "x":row["v_1_x"], "y":row["v_1_y"], 
    #                       "z":row["v_1_z"], 'win': 'Close', 'fire': 'Burn', 'hum': 'None'}
    #             else: 
    #                 record={"time": key, "id":row["id"], "x":row["v_1_x"], "y":row["v_1_y"], 
    #                       "z":row["v_1_z"], 'win': 'Close', 'fire': 'None', 'hum': 'None'}
    #             collection.insert_one(record)
    # print(f"See Sim {Sim}")
    
    #app.run_server(debug=True)
