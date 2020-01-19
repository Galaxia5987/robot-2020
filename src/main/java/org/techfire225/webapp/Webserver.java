package org.techfire225.webapp;

import edu.wpi.first.wpilibj.Filesystem;
import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;
import org.eclipse.jetty.servlet.ServletContextHandler;

import java.io.File;

public class Webserver {
    public Webserver() throws Exception {
        Server server = new Server(5801);

        ServletContextHandler servlets = new ServletContextHandler(ServletContextHandler.SESSIONS);

        servlets.addServlet(RedirectApplet.class, "/");

        servlets.addServlet(StateSocket.class, "/state/socket");
        servlets.addServlet(StateApplets.LatestState.class, "/state/latest");

        ResourceHandler resource_handler = new ResourceHandler();
        resource_handler.setWelcomeFiles(new String[]{"firescope.html"});
        resource_handler.setResourceBase(new File(Filesystem.getDeployDirectory(), "firelog").getPath());

        HandlerList handlers = new HandlerList();
        handlers.setHandlers(new Handler[]{resource_handler, servlets});
        server.setHandler(handlers);

        server.start();
    }
}
