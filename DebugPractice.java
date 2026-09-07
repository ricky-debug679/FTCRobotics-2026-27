public class DebugPractice {

    public static void main(String[] args) {
        int[] numbers = {4, 8, 15, 16, 23, 42};

        int sum = calculateSum(numbers);
        System.out.println("Sum: " + sum);

        double average = calculateAverage(numbers);
        System.out.println("Average: " + average);

        int max = findMax(numbers);
        System.out.println("Max: " + max);
    }

    // Adds up all values in the array
    public static int calculateSum(int[] arr) {
        int total = 0;
        for (int i = 0; i < arr.length; i++) {  // bug: should be i < arr.length
            total += arr[i];
        }
        return total;
    }

    // Returns the average of the array values
    public static double calculateAverage(int[] arr) {
        int sum = calculateSum(arr);
        return (double) sum / arr.length;
    }

    // Returns the largest value in the array
    public static int findMax(int[] arr) {
        int max = arr[0];
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] > max) {
                max = arr[i];
            }
        }
        return max;
    }
}
