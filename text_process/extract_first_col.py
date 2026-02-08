#!/usr/bin/env python3
import sys

def extract_first_column(input_file, output_file):
    with open(input_file, 'r', encoding='utf-8') as fin, \
         open(output_file, 'w', encoding='utf-8') as fout:
        for line_num, line in enumerate(fin, 1):
            line = line.strip()
            if not line:
                # 跳过空行，或可选择写入空行：fout.write('\n')
                continue
            parts = line.split()  # 默认按任意空白符分割（空格、制表符等）
            if parts:
                fout.write(parts[0] + '\n')
            else:
                print(f"警告: 第 {line_num} 行无有效数据", file=sys.stderr)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("用法: python3 extract_first_column.py <输入文件> <输出文件>")
        sys.exit(1)
    
    input_path = sys.argv[1]
    output_path = sys.argv[2]
    
    try:
        extract_first_column(input_path, output_path)
        print(f"成功: 第一列已保存到 '{output_path}'")
    except FileNotFoundError:
        print(f"错误: 找不到输入文件 '{input_path}'", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"错误: {e}", file=sys.stderr)
        sys.exit(1)