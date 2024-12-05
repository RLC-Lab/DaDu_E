# main.py

from executor import Executor


def main():

    # input task list
    print(f"{'Input'.center(100, '-')}")


    # evaluation case
    instruction = 'pick up apple'
    print(instruction)
    print(f"{'Start'.center(100, '-')}")

    executor = Executor()
    executor.execute(instruction)

    print(f"{'Finish'.center(100, '-')}")


if __name__ == "__main__":
    main()

    

