#! /bin/sh

mongo landmarks_study --eval "db.scenes.createIndex({'_meta.name': 1});"
mongo landmarks_study --eval "db.participants.createIndex({'_meta.name': 1});"
