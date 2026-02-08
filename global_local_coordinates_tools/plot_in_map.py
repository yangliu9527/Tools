import pandas as pd
import folium
from pyproj import Proj
from folium.plugins import TimestampedGeoJson

# 1. 数据读取与预处理
def load_data(file_path):
    # 读取制表符分隔的导航数据（列索引根据实际数据调整）
    columns = ['timestamp', 'latitude', 'longitude', 'altitude']
    df = pd.read_csv(file_path, sep='\t', header=None, usecols=[0, 1, 2, 3], names=columns)
    return df[['timestamp', 'latitude', 'longitude', 'altitude']]

# 2. 坐标转换（WGS84转Web墨卡托）
def wgs84_to_web_mercator(df):
    proj = Proj(proj='merc', ellps='WGS84')
    x, y = proj(df['longitude'].values, df['latitude'].values)
    return pd.DataFrame({'x':x, 'y':y})

# 3. 轨迹可视化
def plot_trajectory(df):
    # 创建卫星地图底图
    m = folium.Map(
        location=[df['latitude'].mean(), df['longitude'].mean()],
        zoom_start=17,
        tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
        attr='Google Satellite'
    )
    
    # 添加轨迹线
    coordinates = list(zip(df['latitude'], df['longitude']))
    folium.PolyLine(
        locations=coordinates,
        color='#FF4500',
        weight=5,
        opacity=0.8
    ).add_to(m)
    
    # 添加起终点标记
    folium.Marker(
        coordinates[0], 
        popup=f"起点\n纬度：{coordinates[0][0]:.6f}\n经度：{coordinates[0][1]:.6f}",
        icon=folium.Icon(color='green', icon='play')
    ).add_to(m)
    
    folium.Marker(
        coordinates[-1],
        popup=f"终点\n纬度：{coordinates[-1][0]:.6f}\n经度：{coordinates[-1][1]:.6f}",
        icon=folium.Icon(color='red', icon='stop')
    ).add_to(m)
    
    # 生成时间轴动画（可选）
    # features = [{
    #     "type": "Feature",
    #     "geometry": {
    #         "type": "Point",
    #         "coordinates": [row.longitude, row.latitude],
    #     },
    #     "properties": {
    #         "time": pd.to_datetime(row.timestamp, unit='s').isoformat(),
    #         "popup": f"高度：{row.altitude:.2f}m"
    #     }
    # } for _, row in df.iterrows()]
    
    # TimestampedGeoJson(
    # {"type": "FeatureCollection", "features": features},
    # period="PT1S",
    # transition_time=100,
    # auto_play=True
    # ).add_to(m)
    
    return m

# 主程序
if __name__ == "__main__":
    df = load_data("navigation_data.txt")  # 替换为实际文件路径
    m = plot_trajectory(df)
    m.save("navigation_trajectory.html")
