#!/usr/bin/python
__author__ = 'nick'

import rethinkdb as r
import json
from classes import *

classes = ['target', 'vehicle', 'position3d']


def query_table_existence(db, table, connection):
    tables = r.db(db).table_list().run(connection)
    if table in tables:
        return True
    return False


def get_instance_by_UID(uid, db, table, connection):
    return r.db(db).table(table).get(uid).run(connection)


def insert_instance(db, table, connection, instance):
    # if type(instance).__name__ in classes:
    r.db(db).table(table).insert(instance.getJSON()).run(connection)




