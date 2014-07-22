import re
import csv
from sklearn.feature_extraction import DictVectorizer




with open('/Users/carrier24sg/Documents/work/kdd-cup/donations.csv', 'r') as f:
    reader = csv.reader(f, delimiter=',')

    fully_funded_projects= set()
    not_fully_funded_projects = set()
    
    p2c = {}

    for i,row in enumerate(reader):
        if i == 0:
            header = row
        else:
            item = dict(zip(header,row))
            for k,v in item.items():
                try:
                    item[k] = float(v)
                except ValueError:
                    pass
            
                project_id = item['projectid']
                if project_id in p2c:
                    p2c[project_id] = p2c[project_id] + item['donation_to_project']
                else:
                    p2c[project_id] = item['donation_to_project']


with open('/Users/carrier24sg/Documents/work/kdd-cup/projects.csv', 'r') as f:
    reader = csv.reader(f, delimiter=',')
    records = []
    
    project_set = set()
    school_set = set()
    donors_set = set()

    for i,row in enumerate(reader):
        if i == 0:
            header = row
        if i == 20:
            break
        else:
            item = dict(zip(header,row))
            for k,v in item.items():
                try:
                    item[k] = float(v)
                except ValueError:
                    pass

            if item['fully_funded'] == 't':
                donations = p2c[item['project_id']]
                print 'fully funded == total donations? %s' %(donations == 
                


