import io.ebinoma.rcljava.RCLJava;
import io.ebinoma.rcljava.Node;
import io.ebinoma.rcljava.Publisher;

public class Talker {
    public static void main(String[] args) throws InterruptedException {
        RCLJava.rcljavaInit();

        Node node = RCLJava.createNode("talker");

        Publisher<std_msgs.msg.String> chatter_pub =
            node.<std_msgs.msg.String>createPublisher(std_msgs.msg.String.class, "chatter");

//        Publisher<std_msgs.msg.Int32> chatter_pub =
//            node.<std_msgs.msg.Int32>createPublisher(std_msgs.msg.Int32.class, "chatter");

        std_msgs.msg.String msg = new std_msgs.msg.String();

//        std_msgs.msg.Int32 msg = new std_msgs.msg.Int32();

        int i = 1;

        while(true) {
            msg.data = "Hello World: " + i;
            // msg.data = i;
            i++;
            System.out.println("Publishing: \"" + msg.data + "\"");
            chatter_pub.publish(msg);
            Thread.sleep(1000);
        }
    }
}
