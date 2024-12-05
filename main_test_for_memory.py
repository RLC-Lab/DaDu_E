from executor_test_memory import Executor

def main():
    # 设置是否使用 memory
    use_memory = True  # 直接设置为 True

    # 输入任务列表
    print(f"{'Input'.center(100, '-')}")

    # 测试任务指令
    instruction = 'pick up apple'
    print(instruction)
    print(f"{'Start'.center(100, '-')}")

    # 创建 Executor 实例
    executor = Executor(use_memory=use_memory)
    executor.execute(instruction)

    print(f"{'Finish'.center(100, '-')}")

if __name__ == "__main__":
    main()
