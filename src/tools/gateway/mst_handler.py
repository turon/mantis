#!/usr/bin/python
import time

def mst_handler(new_node_dict, nodedict):

    if nodedict.has_key('packets'):
        nodedict['packets'] = nodedict['packets'] + 1
    else:
        nodedict['packets'] = 1
    
    if nodedict.has_key('first_tv_h'):
        tv = time.time ()
        tv = int (tv)
        total_time = tv - nodedict['first_tv_h']
        if total_time > 0:
            packets_s = nodedict['packets'] / total_time
            nodedict['pkt/s'] = packets_s

            if nodedict.has_key('dropped'):
                dropped_s = nodedict['dropped'] / total_time
                nodedict['drp/s'] = dropped_s
    else:
        tv = time.time ()
        tv = int (tv)
        nodedict['first_tv_h'] = tv
    
    if nodedict.has_key('seq'):
        oldseq = nodedict['seq']
        seq = new_node_dict['seq']

        if seq - oldseq - 1 > 0:
            try:
                nodedict['dropped'] = nodedict['dropped'] + 1
            except:
                nodedict['dropped'] = 1
            
        
    
def mst_tally(nodelist):
    average_dict = {}
    total_dict = {}

    nodecount = 0
    packetcount = 0
    droppedcount = 0
    pkts_scount = 0
    drp_scount = 0
    first_tv = 0
    for nodeid in nodelist:
        if nodeid == 'average':
            continue
        if nodeid == 'total':
            continue
        nodecount = nodecount + 1
        node = nodelist[nodeid]['9']
        if first_tv == 0:
            first_tv = node['first_tv_h']
        else:
            if node['first_tv_h'] < first_tv:
                first_tv = node['first_tv_h']

        packetcount = packetcount + node['packets']

        if node.has_key('pkt/s'):
            pkts_scount = pkts_scount + node['pkt/s']
        if node.has_key('dropped'):
            droppedcount = droppedcount + node['dropped']

        if node.has_key('drp/s'):
            drp_scount = drp_scount + node['drp/s']
        
    tv = time.time ()
    tv = int (tv)
    total_time = tv - first_tv
    if total_time == 0:                
        return

    if nodecount == 0:
        return
    
    average_dict = {'9':{ 'packets': packetcount/nodecount, 'dropped': droppedcount/total_time, 'pkt/s': pkts_scount / nodecount, 'drp/s': drp_scount/nodecount}}
    total_dict = {'9':{ 'packets': packetcount, 'dropped': droppedcount, 'pkt/s': packetcount/total_time, 'drp/s': droppedcount/total_time}}
    nodelist['average'] = average_dict
    nodelist['total'] = total_dict
