
def quicksort(nums):
   if len(nums) <= 1:
       return nums
   else:
       if len(nums)%2 == 0:
           q = nums[len(nums)//2]
       else:
           q = nums[len(nums)//2 + 1]
       s_nums = []
       m_nums = []
       e_nums = []
       for n in nums:
           if n < q:
               s_nums.append(n)
           elif n > q:
               m_nums.append(n)
           else:
               e_nums.append(n)
       return quicksort(s_nums) + e_nums + quicksort(m_nums)
nums = list(map(int, input().split()))
print(*quicksort(nums))
