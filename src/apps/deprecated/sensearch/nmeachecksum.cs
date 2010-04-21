using System;
using System.Text;

namespace main
{
   public class Program
   {
      public static bool IsValid(string sentence)
      {
	 // Compare the characters after the asterisk to the calculation
	 return sentence.Substring(sentence.IndexOf("*") + 1) == 
	    GetChecksum(sentence);
      }

     // Calculates the checksum for a sentence
      public static string GetChecksum(string sentence)
      {
	 // Loop through all chars to get a checksum
	 int Checksum = 0;
	 foreach (char Character in sentence)
	 {
	    if (Character == '$')
	    {
		continue;
	    }
	    else if (Character == '*')
	    {
	       // Stop processing before the asterisk
	       break;
	    }
	    else
	    {
	       Checksum = Checksum ^ Convert.ToByte(Character);
	    }
	 }
	 
	 // Return the checksum formatted as a two-character hexadecimal
	 return Checksum.ToString("X2");
      }

      public static void Main(string[] args)
      {
	 foreach(string s in args)
	 {
	    if (s.IndexOf('*') != -1)
	    {
	       if (IsValid(s))
		  Console.WriteLine("{0} is valid.", s);
	       else
		  Console.WriteLine("{0} should be {1}", s, GetChecksum(s));
	    
	    }
	    else
	    {
	       Console.WriteLine("{0}*{1}", s, GetChecksum(s));
	    }
	    
	 }
	 
      }
      
   }
      
}
