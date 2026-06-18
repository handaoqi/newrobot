import os
import json
import shutil
from django.core.management.base import BaseCommand
from django.conf import settings
from monitoring.models import MapData


class Command(BaseCommand):
    help = '迁移现有地图数据到Django数据库'

    def handle(self, *args, **options):
        # 源数据目录
        source_data_dir = r'C:\Users\talent\Documents\trae_projects\建图页面\backend\data'
        maps_json_path = os.path.join(source_data_dir, 'maps.json')
        maps_dir = os.path.join(source_data_dir, 'maps')

        # 目标目录
        media_maps_dir = os.path.join(settings.MEDIA_ROOT, 'maps')
        media_thumbnails_dir = os.path.join(settings.MEDIA_ROOT, 'maps', 'thumbnails')

        # 确保目标目录存在
        os.makedirs(media_maps_dir, exist_ok=True)
        os.makedirs(media_thumbnails_dir, exist_ok=True)

        # 读取 maps.json
        with open(maps_json_path, 'r', encoding='utf-8') as f:
            maps_data = json.load(f)

        for map_id, map_info in maps_data.items():
            self.stdout.write(f'正在迁移地图: {map_id}')

            # 源目录
            source_map_dir = os.path.join(maps_dir, map_id)

            # 目标目录
            target_map_dir = os.path.join(media_maps_dir, map_id)
            os.makedirs(target_map_dir, exist_ok=True)

            # 复制文件
            pgm_path = None
            yaml_path = None
            thumbnail_path = None

            for filename in map_info.get('files', []):
                source_file = os.path.join(source_map_dir, filename)
                target_file = os.path.join(target_map_dir, filename)

                if os.path.exists(source_file):
                    shutil.copy2(source_file, target_file)
                    self.stdout.write(f'  复制文件: {filename}')

                    if filename == 'map.pgm':
                        pgm_path = f'maps/{map_id}/{filename}'
                    elif filename == 'map.yaml':
                        yaml_path = f'maps/{map_id}/{filename}'
                    elif filename == 'map_preview.png':
                        # 缩略图单独存放
                        thumbnail_target = os.path.join(media_thumbnails_dir, f'{map_id}.png')
                        shutil.copy2(source_file, thumbnail_target)
                        thumbnail_path = f'maps/thumbnails/{map_id}.png'

            # 创建数据库记录
            map_data = MapData(
                id=None,
                name=map_info.get('filename', map_id),
                resolution=0.05,
                active=False,
                description=f'迁移自原有系统: {map_id}',
            )

            if pgm_path:
                map_data.pgm_file.name = pgm_path
            if yaml_path:
                map_data.yaml_file.name = yaml_path
            if thumbnail_path:
                map_data.thumbnail.name = thumbnail_path

            map_data.save()
            self.stdout.write(f'  创建数据库记录成功')

        self.stdout.write(self.style.SUCCESS('地图数据迁移完成！'))